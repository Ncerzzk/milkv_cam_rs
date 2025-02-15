mod cvi_isp;
mod cvi_sns_ctrl;
mod cvi_sys;
mod cvi_vb;
mod cvi_vi;
mod cvi_comm_vi;
mod cvi_ae;
mod cvi_awb;
mod cvi_bin;
mod cvi_venc;
mod sensor;

use clap::Parser;
use cvi_vb::VB_POOL_CONFIG_S;
use std::{os::raw::c_void, io::{Read, Write}, ptr::null_mut, mem::ManuallyDrop,sync::{LazyLock, atomic::AtomicBool}};

use crate::{
    cvi_isp::{CVI_MIPI_SetMipiReset, CVI_MIPI_SetSensorReset, CVI_MIPI_SetMipiAttr, CVI_MIPI_SetSensorClock, CVI_ISP_SetBindAttr, CVI_ISP_MemInit, ISP_PUB_ATTR_S, },
    cvi_sns_ctrl::{
        combo_dev_attr_s__bindgen_ty_1, img_size_s, mclk_pll_s, mipi_dev_attr_s,
        SNS_COMBO_DEV_ATTR_S, dphy_s, mipi_demux_info_s, manual_wdr_attr_s, stSnsGc2083_Obj, combo_dev_attr_s, ISP_INIT_ATTR_S, ISP_SNS_COMMBUS_U, ALG_LIB_S,
        ISP_SENSOR_EXP_FUNC_S, ISP_CMOS_SENSOR_IMAGE_MODE_S, ISP_BIND_ATTR_S
    },
    cvi_sys::{CVI_SYS_Exit, CVI_SYS_Init},
    cvi_vb::{CVI_VB_Exit, CVI_VB_Init, CVI_VB_SetConfig, VB_CONFIG_S},
    cvi_vi::{
        CVI_VI_EnableDev, CVI_VI_SetDevAttr, CVI_VI_SetDevNum, VI_DEV_ATTR_S, VI_SYNC_CFG_S,
        VI_TIMING_BLANK_S, VI_WDR_ATTR_S, _SIZE_S, CVI_VI_CreatePipe,
        VI_PIPE_ATTR_S, _PIXEL_FORMAT_E_PIXEL_FORMAT_RGB_BAYER_12BPP, _DATA_BITWIDTH_E_DATA_BITWIDTH_12, FRAME_RATE_CTRL_S, CVI_VI_StartPipe
    }, cvi_ae::CVI_AE_Register, cvi_awb::CVI_AWB_Register
};

macro_rules! err_check {  
    ($expr:expr) => { 
        let _tmp = $expr;
        if(_tmp != 0){
            println!("error at {}, return:{:x}",line!(), _tmp);
        } 
    };  
}

static AE_LIB:LazyLock<ALG_LIB_S>= LazyLock::new(||{
    let mut ae_lib_name = [0 as u8;20];

    unsafe{
        std::ptr::copy_nonoverlapping("cvi_ae_lib".as_ptr(), ae_lib_name.as_mut_ptr() as *mut u8, "cvi_ae_lib".len());
    }

    ALG_LIB_S{
        s32Id: 0,
        acLibName: ae_lib_name
    }
});

static AWB_LIB:LazyLock<ALG_LIB_S>= LazyLock::new(||{
    let mut awb_lib_name = [0 as u8;20];
    
    unsafe{
        std::ptr::copy_nonoverlapping("cvi_awb_lib".as_ptr(), awb_lib_name.as_mut_ptr() as *mut u8, "cvi_awb_lib".len());
    }

    ALG_LIB_S{
        s32Id: 0,
        acLibName: awb_lib_name
    }
});


#[no_mangle] 
pub extern "C" fn __sync_fetch_and_and_1(p:*mut c_void, v:u32) -> u32{
    unsafe{
        std::sync::atomic::AtomicU32::from_ptr(p as *mut u32).fetch_and(v, std::sync::atomic::Ordering::Relaxed)
    }
}

fn import_bin(){
    unsafe{
        let mut bin_name:[u8;256] = [0;256];
        cvi_bin::CVI_BIN_GetBinName(&mut bin_name as *mut u8);
        let bin_name_str = std::str::from_utf8(std::ffi::CStr::from_bytes_until_nul(&bin_name).unwrap().to_bytes()).unwrap();
        let mut f = std::fs::File::options().read(true).open(bin_name_str).unwrap();
        let mut buf = Vec::new();
        let file_size = f.read_to_end(&mut buf).unwrap();

        println!("name:{} size:{:x}",&bin_name_str,file_size);

        err_check!(cvi_bin::CVI_BIN_ImportBinData(buf.as_mut_ptr(),file_size as u32));
    }
}

unsafe fn vi_venc_init(){
    let mut temp = cvi_sys::MMF_VERSION_S { version: [0; 128] };

    cvi_sys::CVI_SYS_GetVersion(&mut temp as *mut cvi_sys::MMF_VERSION_S);

    let mut log_conf = cvi_sys::LOG_LEVEL_CONF_S{
        enModId: 28,
        s32Level: 7,
        cModName: [0;16],
    };
    cvi_sys::CVI_LOG_SetLevelConf(&mut log_conf as *mut cvi_sys::LOG_LEVEL_CONF_S);
    CVI_VI_SetDevNum(1);

    CVI_SYS_Exit();
    CVI_VB_Exit();

    /*
    [VB_POOL_CONFIG_S {
            u32BlkSize: ((1920 * 10 + 7) / 8 * 1080 + 0x3F) & (!0x3F),
            u32BlkCnt: 4,
            enRemapMode: 2, // 2 is VB_REMAP_MODE_CACHED
            acName: [0; 32],
        }; 16]
     */
    let mut vb_config = VB_CONFIG_S {
        u32MaxPoolCnt: 1,
        astCommPool: [VB_POOL_CONFIG_S {
            u32BlkSize:0,
            u32BlkCnt:0,
            enRemapMode: 2, // 2 is VB_REMAP_MODE_CACHED
            acName: [0; 32],
        }; 16],
    };

    vb_config.astCommPool[0].u32BlkCnt = 4;
    vb_config.astCommPool[0].u32BlkSize = 3133440;//((1920 * 10 + 7) / 8 * 1080 + 0x3F) & (!0x3F);


    CVI_VB_SetConfig(&vb_config as *const VB_CONFIG_S);

    err_check!(CVI_VB_Init());
    err_check!(CVI_SYS_Init());

    println!("cvi sys inited.");

    let mut isp_init_attr = ISP_INIT_ATTR_S{
        u32ExpTime: 0,
        u32AGain: 0,
        u32DGain: 0,
        u32ISPDGain: 0,
        u32Exposure: 0,
        u32LinesPer500ms: 0,
        u32PirisFNO: 0,
        u16WBRgain: 0,
        u16WBGgain: 0,
        u16WBBgain: 0,
        u16SampleRgain: 0,
        u16SampleBgain: 0,
        u16UseHwSync: 0,
        enGainMode: 0,
        enL2SMode: 0,
        enSnsBdgMuxMode: 0,
    };

    stSnsGc2083_Obj.pfnSetInit.unwrap()(0, &mut isp_init_attr as *mut ISP_INIT_ATTR_S);
    stSnsGc2083_Obj.pfnSetBusInfo.unwrap()(0,ISP_SNS_COMMBUS_U{
        s8I2cDev:{1}
    });

    stSnsGc2083_Obj.pfnPatchI2cAddr.unwrap()(37);

    stSnsGc2083_Obj.pfnRegisterCallback.unwrap()(0, &mut AE_LIB.clone() as *mut ALG_LIB_S, &mut AWB_LIB.clone() as *mut ALG_LIB_S);

    let sns_dev_attr: combo_dev_attr_s = SNS_COMBO_DEV_ATTR_S {
        input_mode: 0, //INPUT_MODE_MIPI
        mac_clk: 0,    // RX_MAC_CLK_200M
        mclk: mclk_pll_s {
            cam: 0,
            freq: 4, // CAMPLL_FREQ_24M
        },
        __bindgen_anon_1: combo_dev_attr_s__bindgen_ty_1 {
            mipi_attr: mipi_dev_attr_s {
                raw_data_type: 1, // RAW_DATA_10BIT
                lane_id:[3, 2, 4, -1, -1],
                wdr_mode: 0, // CVI_MIPI_WDR_MODE_NONE
                data_type: [0;2],
                pn_swap: [0;5],
                dphy: dphy_s{
                    enable: 0,
                    hs_settle: 0,
                },
                demux: mipi_demux_info_s{
                    demux_en: 0,
                    vc_mapping: [0;4],
                },
            },
        },
        devno: 0,
        img_size: img_size_s {
            width: 1920,
            height: 1080,
        },
        wdr_manu: manual_wdr_attr_s{
            manual_en: 0,
            l2s_distance: 0,
            lsef_length: 0,
            discard_padding_lines: 0,
            update: 0,
        },
    };

    println!("register callback.");

    //stSnsGc2083_Obj.pfnGetRxAttr.unwrap()(0,&mut sns_dev_attr as *mut combo_dev_attr_s);
    let mut sensor_funcs = ISP_SENSOR_EXP_FUNC_S{
        pfn_cmos_sensor_init: None,
        pfn_cmos_sensor_exit: None,
        pfn_cmos_sensor_global_init: None,
        pfn_cmos_set_image_mode: None,
        pfn_cmos_set_wdr_mode: None,
        pfn_cmos_get_isp_default: None,
        pfn_cmos_get_isp_black_level: None,
        pfn_cmos_get_sns_reg_info: None,
    };
    
    let mut sensor_mode = ISP_CMOS_SENSOR_IMAGE_MODE_S{
        u16Width: 1920,
        u16Height: 1080,
        f32Fps: 30.0,
        u8SnsMode:0,
    };

    stSnsGc2083_Obj.pfnExpSensorCb.unwrap()(&mut sensor_funcs as *mut ISP_SENSOR_EXP_FUNC_S);

    sensor_funcs.pfn_cmos_set_image_mode.unwrap()(0, &mut sensor_mode as *mut ISP_CMOS_SENSOR_IMAGE_MODE_S);
    sensor_funcs.pfn_cmos_set_wdr_mode.unwrap()(0,0);


    let vi_attr = VI_DEV_ATTR_S {
        enIntfMode: cvi_vi::_VI_INTF_MODE_E_VI_MODE_MIPI, // VI_MODE_MIPI
        enWorkMode: cvi_vi::_VI_WORK_MODE_E_VI_WORK_MODE_1Multiplex, // VI_WORK_MODE_1Multiplex
        enScanMode: cvi_vi::_VI_SCAN_MODE_E_VI_SCAN_PROGRESSIVE, // VI_SCAN_PROGRESSIVE
        as32AdChnId: [-1; 4],
        enDataSeq: cvi_vi::_VI_YUV_DATA_SEQ_E_VI_DATA_SEQ_YUYV, // VI_DATA_SEQ_YUYV
        stSynCfg: VI_SYNC_CFG_S {
            enVsync: cvi_vi::_VI_VSYNC_E_VI_VSYNC_PULSE,         // VI_VSYNC_PULSE
            enVsyncNeg: cvi_vi::_VI_VSYNC_NEG_E_VI_VSYNC_NEG_LOW,      // VI_VSYNC_NEG_LOW
            enHsync: cvi_vi::_VI_HSYNC_E_VI_HSYNC_VALID_SINGNAL,         // VI_HSYNC_VALID_SINGNAL
            enHsyncNeg: cvi_vi::_VI_HSYNC_NEG_E_VI_HSYNC_NEG_HIGH,      // VI_HSYNC_NEG_HIGH
            enVsyncValid: cvi_vi::_VI_VSYNC_VALID_E_VI_VSYNC_VALID_SIGNAL,    // VI_VSYNC_VALID_SIGNAL
            enVsyncValidNeg: cvi_vi::_VI_VSYNC_VALID_NEG_E_VI_VSYNC_VALID_NEG_HIGH, // VI_VSYNC_VALID_NEG_HIGH
            stTimingBlank: VI_TIMING_BLANK_S {
                u32HsyncHfb: 0,
                u32HsyncAct: 1920,
                u32HsyncHbb: 0,
                u32VsyncVfb: 0,
                u32VsyncVact: 1080,
                u32VsyncVbb: 0,
                u32VsyncVbfb: 0,
                u32VsyncVbact: 0,
                u32VsyncVbbb: 0,
            },
        },
        enInputDataType: cvi_vi::_VI_DATA_TYPE_E_VI_DATA_TYPE_RGB, // VI_DATA_TYPE_RGB
        stSize: _SIZE_S {
            u32Width: 1920,
            u32Height: 1080,
        },
        stWDRAttr: VI_WDR_ATTR_S {
            enWDRMode: 0,
            u32CacheLine: 1080,
            bSyntheticWDR: 0,
        },
        enBayerFormat: cvi_vi::_BAYER_FORMAT_E_BAYER_FORMAT_RG, 
        chn_num: 0,
        snrFps: 0,
    };

    CVI_VI_SetDevAttr(0, &vi_attr as *const VI_DEV_ATTR_S);

    err_check!(CVI_VI_EnableDev(0));
    

    CVI_MIPI_SetSensorReset(0, 1);

    CVI_MIPI_SetMipiReset(0, 1);

    CVI_MIPI_SetMipiAttr(0, &sns_dev_attr as *const SNS_COMBO_DEV_ATTR_S  as *const c_void );

    CVI_MIPI_SetSensorClock(0,1);

    std::thread::sleep(std::time::Duration::from_micros(20));
    CVI_MIPI_SetSensorReset(0,0);

    println!("finished sensor reset!");

    stSnsGc2083_Obj.pfnSnsProbe.unwrap()(0);

    println!("sensor probed");

    let vi_pipe_attr = VI_PIPE_ATTR_S{
        enPipeBypassMode: 0,
        bYuvSkip: 0,
        bIspBypass: 0,
        u32MaxW: 1920,
        u32MaxH: 1080,
        enPixFmt: _PIXEL_FORMAT_E_PIXEL_FORMAT_RGB_BAYER_12BPP,
        enCompressMode: 1,
        enBitWidth: _DATA_BITWIDTH_E_DATA_BITWIDTH_12,
        bNrEn: 1,
        bSharpenEn: 0,
        stFrameRate: FRAME_RATE_CTRL_S{
            s32SrcFrameRate: -1,
            s32DstFrameRate: -1,
        } ,
        bDiscardProPic: 0,
        bYuvBypassPath: 0,
    };

    err_check!(CVI_VI_CreatePipe(0, &vi_pipe_attr as *const VI_PIPE_ATTR_S));
    err_check!(CVI_VI_StartPipe(0));

    println!("created vi pipe");

    err_check!(CVI_AE_Register(0, &mut AE_LIB.clone() as *mut ALG_LIB_S as *mut cvi_ae::_ALG_LIB_S));
    err_check!(CVI_AWB_Register(0, &mut AWB_LIB.clone() as *mut ALG_LIB_S as *mut cvi_awb::_ALG_LIB_S));

    let isp_bind_attr = ISP_BIND_ATTR_S{
        sensorId: 0,
        stAeLib: AE_LIB.clone(),
        stAfLib: ALG_LIB_S{
            s32Id: 0,
            acLibName: [0;20],
        },
        stAwbLib: AWB_LIB.clone(),
    };

    err_check!(CVI_ISP_SetBindAttr(0, &isp_bind_attr as *const ISP_BIND_ATTR_S as *const cvi_isp::_ISP_BIND_ATTR_S));

    println!("cvi isp mem inited prepare.");
    err_check!(CVI_ISP_MemInit(0));
    println!("cvi isp mem inited.");

    let isp_pub_attr = ISP_PUB_ATTR_S{
        stWndRect: cvi_isp::_RECT_S{
            s32X: 0,
            s32Y: 0,
            u32Width: 1920,
            u32Height: 1080,
        },
        stSnsSize: cvi_isp::_SIZE_S{
            u32Width: 1920,
            u32Height: 1080,
        },
        f32FrameRate: 30.0,
        enBayer: cvi_isp::_ISP_BAYER_FORMAT_E_BAYER_RGGB,
        enWDRMode: 0,
        u8SnsMode: 0,
    };

    cvi_isp::CVI_ISP_SetPubAttr(0, &isp_pub_attr as *const ISP_PUB_ATTR_S);

    let mut isp_static_cfg: cvi_isp::_ISP_STATISTICS_CFG_S =  cvi_isp::ISP_STATISTICS_CFG_S{
        unKey: cvi_isp::ISP_STATISTICS_CTRL_U{
           u64Key:0x1f 
        },
        stAECfg: cvi_isp::ISP_AE_STATISTICS_CFG_S{
            bHisStatisticsEnable: 0,
            stCrop: [cvi_isp::ISP_AE_CROP_S{
                bEnable: 0,
                u16X: 0,
                u16Y: 0,
                u16W: 1920,
                u16H: 1080,
            }],
            stFaceCrop: [cvi_isp::ISP_AE_FACE_CROP_S{
                bEnable: 0,
                u16X: 0,
                u16Y: 0,
                u16W: 0,
                u16H: 0,
            };4],
            fast2A_ena: 0,
            fast2A_ae_low: 0,
            fast2A_ae_high: 0,
            fast2A_awb_top: 0,
            fast2A_awb_bot: 0,
            over_exp_thr: 0,
            au8Weight: [[1;17];15],
        },
        stWBCfg: cvi_isp::ISP_WB_STATISTICS_CFG_S{
            enAWBSwitch: 0,
            u16ZoneRow: 30,
            u16ZoneCol: 34,
            u16ZoneBin: 0,
            au16HistBinThresh: [0;4],
            u16WhiteLevel: 4095,
            u16BlackLevel: 0,
            u16CbMax: 0,
            u16CbMin: 0,
            u16CrMax: 0,
            u16CrMin: 0,
            stCrop: cvi_isp::ISP_AWB_CROP_S{
                bEnable: 0,
                u16X: 0,
                u16Y: 0,
                u16W: 1920,
                u16H: 1080,
            },
        },
        stFocusCfg: cvi_isp::ISP_FOCUS_STATISTICS_CFG_S{
            stConfig: cvi_isp::ISP_AF_CFG_S{
                bEnable: 0,
                u16Hwnd: 0,
                u16Vwnd: 0,
                u8HFltShift: 0,
                s8HVFltLpCoeff: [0;5],
                stRawCfg: cvi_isp::ISP_AF_RAW_CFG_S{
                    PreGammaEn: 0,
                    PreGammaTable: [0;256],
                },
                stPreFltCfg: cvi_isp::ISP_AF_PRE_FILTER_CFG_S{
                    PreFltEn: 0,
                },
                stCrop: cvi_isp::ISP_AF_CROP_S{
                    bEnable: 0,
                    u16X: 0,
                    u16Y: 0,
                    u16W: 0,
                    u16H: 0,
                },
                H0FltCoring: 0,
                H1FltCoring: 0,
                V0FltCoring: 0,
                u16HighLumaTh: 0,
                u8ThLow: 0,
                u8ThHigh: 0,
                u8GainLow: 0,
                u8GainHigh: 0,
                u8SlopLow: 0,
                u8SlopHigh: 0,
            },
            stHParam_FIR0: cvi_isp::ISP_AF_H_PARAM_S{
                s8HFltHpCoeff: [0;5],
            },
            stHParam_FIR1: cvi_isp::ISP_AF_H_PARAM_S{
                s8HFltHpCoeff: [0;5],
            },
            stVParam_FIR: cvi_isp::ISP_AF_V_PARAM_S{
                s8VFltHpCoeff: [0;3],
            },
        },
    };

    err_check!(cvi_isp::CVI_ISP_GetStatisticsConfig(0, &mut isp_static_cfg as *mut cvi_isp::_ISP_STATISTICS_CFG_S));

    isp_static_cfg.stAECfg.stCrop[0].bEnable = 0;
    isp_static_cfg.stAECfg.stCrop[0].u16X = 0;
    isp_static_cfg.stAECfg.stCrop[0].u16Y = 0;
    isp_static_cfg.stAECfg.stCrop[0].u16W = 1920;
    isp_static_cfg.stAECfg.stCrop[0].u16H = 1080;        

    std::ptr::write_bytes(isp_static_cfg.stAECfg.au8Weight.as_mut_ptr() as *mut u8, 1, size_of_val(&isp_static_cfg.stAECfg.au8Weight));

    isp_static_cfg.stWBCfg.u16ZoneRow = cvi_isp::AWB_ZONE_ORIG_ROW as u16;
    isp_static_cfg.stWBCfg.u16ZoneCol = cvi_isp::AWB_ZONE_ORIG_COLUMN as u16;
    isp_static_cfg.stWBCfg.stCrop.bEnable = 0;
    isp_static_cfg.stWBCfg.stCrop.u16X = 0;
    isp_static_cfg.stWBCfg.stCrop.u16Y = 0;
    isp_static_cfg.stWBCfg.stCrop.u16W = 1920;
    isp_static_cfg.stWBCfg.stCrop.u16H = 1080;
    isp_static_cfg.stWBCfg.u16BlackLevel = 0;
    isp_static_cfg.stWBCfg.u16WhiteLevel = 4095;
    isp_static_cfg.stFocusCfg.stConfig.bEnable = 1;
    isp_static_cfg.stFocusCfg.stConfig.u8HFltShift = 1;
    isp_static_cfg.stFocusCfg.stConfig.s8HVFltLpCoeff[0] = 1;
    isp_static_cfg.stFocusCfg.stConfig.s8HVFltLpCoeff[1] = 2;
    isp_static_cfg.stFocusCfg.stConfig.s8HVFltLpCoeff[2] = 3;
    isp_static_cfg.stFocusCfg.stConfig.s8HVFltLpCoeff[3] = 5;
    isp_static_cfg.stFocusCfg.stConfig.s8HVFltLpCoeff[4] = 10;
    isp_static_cfg.stFocusCfg.stConfig.stRawCfg.PreGammaEn = 0;
    isp_static_cfg.stFocusCfg.stConfig.stPreFltCfg.PreFltEn = 1;
    isp_static_cfg.stFocusCfg.stConfig.u16Hwnd = 17;
    isp_static_cfg.stFocusCfg.stConfig.u16Vwnd = 15;
    isp_static_cfg.stFocusCfg.stConfig.stCrop.bEnable = 0;
    // AF offset and size has some limitation.
    isp_static_cfg.stFocusCfg.stConfig.stCrop.u16X = 8;
    isp_static_cfg.stFocusCfg.stConfig.stCrop.u16Y = 2;
    isp_static_cfg.stFocusCfg.stConfig.stCrop.u16W = 1920-8*2;
    isp_static_cfg.stFocusCfg.stConfig.stCrop.u16H = 1080-2*2;
    //Horizontal HP0
    isp_static_cfg.stFocusCfg.stHParam_FIR0.s8HFltHpCoeff[0] = 0;
    isp_static_cfg.stFocusCfg.stHParam_FIR0.s8HFltHpCoeff[1] = 0;
    isp_static_cfg.stFocusCfg.stHParam_FIR0.s8HFltHpCoeff[2] = 13;
    isp_static_cfg.stFocusCfg.stHParam_FIR0.s8HFltHpCoeff[3] = 24;
    isp_static_cfg.stFocusCfg.stHParam_FIR0.s8HFltHpCoeff[4] = 0;
    //Horizontal HP1
    isp_static_cfg.stFocusCfg.stHParam_FIR1.s8HFltHpCoeff[0] = 1;
    isp_static_cfg.stFocusCfg.stHParam_FIR1.s8HFltHpCoeff[1] = 2;
    isp_static_cfg.stFocusCfg.stHParam_FIR1.s8HFltHpCoeff[2] = 4;
    isp_static_cfg.stFocusCfg.stHParam_FIR1.s8HFltHpCoeff[3] = 8;
    isp_static_cfg.stFocusCfg.stHParam_FIR1.s8HFltHpCoeff[4] = 0;
    //Vertical HP
    isp_static_cfg.stFocusCfg.stVParam_FIR.s8VFltHpCoeff[0] = 13;
    isp_static_cfg.stFocusCfg.stVParam_FIR.s8VFltHpCoeff[1] = 24;
    isp_static_cfg.stFocusCfg.stVParam_FIR.s8VFltHpCoeff[2] = 0;
    isp_static_cfg.unKey.u64Key = 0x1f;

    cvi_isp::CVI_ISP_SetStatisticsConfig(0,&isp_static_cfg as *const cvi_isp::_ISP_STATISTICS_CFG_S);

    err_check!(cvi_isp::CVI_ISP_Init(0));

    import_bin();
    let _isp_thread = std::thread::spawn(||{
        err_check!(cvi_isp::CVI_ISP_Run(0));
        println!("cvi isp run thread finished.");
    });

    let mut vi_chn_attr = cvi_vi::VI_CHN_ATTR_S{
        stSize: cvi_vi::SIZE_S{
            u32Width: 1920,
            u32Height: 1080,
        },
        enPixelFormat: cvi_vi::_PIXEL_FORMAT_E_PIXEL_FORMAT_NV21,
        enDynamicRange: 0,
        enVideoFormat: 0,
        enCompressMode: 1,
        bMirror: 0,
        bFlip: 0,
        u32Depth: 0,
        stFrameRate: cvi_vi::FRAME_RATE_CTRL_S{
            s32SrcFrameRate: -1,
            s32DstFrameRate: -1,
        },
        u32BindVbPool: u32::MAX,
    };
    cvi_vi::CVI_VI_SetChnAttr(0, 0, &mut vi_chn_attr as *mut cvi_vi::VI_CHN_ATTR_S);
    std::thread::sleep(std::time::Duration::from_millis(10));
    err_check!(cvi_vi::CVI_VI_EnableChn(0, 0));


    let mut venc_param = cvi_venc::VENC_PARAM_MOD_S{
        enVencModType: 0,
        __bindgen_anon_1: cvi_venc::_VENC_MODPARAM_S__bindgen_ty_1{
            stH265eModParam: cvi_venc::VENC_MOD_H265E_S{
                u32OneStreamBuffer: 0,
                u32H265eMiniBufMode: 0,
                u32H265ePowerSaveEn: 0,
                enH265eVBSource: 0,
                bQpHstgrmEn: 0,
                u32UserDataMaxLen: 0,
                bSingleEsBuf: 0,
                u32SingleEsBufSize: 0,
                enRefreshType: 0,
            }
        },
    };

    cvi_venc::CVI_VENC_GetModParam(&mut venc_param as *mut cvi_venc::VENC_PARAM_MOD_S);
    venc_param.enVencModType = cvi_venc::_VENC_MODTYPE_E_MODTYPE_H265E;
    venc_param.__bindgen_anon_1.stH265eModParam.enH265eVBSource = cvi_venc::_VB_SOURCE_E_VB_SOURCE_PRIVATE;
    venc_param.__bindgen_anon_1.stH265eModParam.u32UserDataMaxLen = 3072;
    venc_param.__bindgen_anon_1.stH265eModParam.bSingleEsBuf = 0;
    venc_param.__bindgen_anon_1.stH265eModParam.u32SingleEsBufSize = 0; // pCic->singleEsBufSize_h265e;
    venc_param.__bindgen_anon_1.stH265eModParam.enRefreshType = 0;

    cvi_venc::CVI_VENC_SetModParam(&mut venc_param as *mut cvi_venc::VENC_PARAM_MOD_S);

    let venc_attr = cvi_venc::VENC_CHN_ATTR_S{
        stVencAttr: cvi_venc::VENC_ATTR_S{
            enType:cvi_venc::PAYLOAD_TYPE_E_PT_H265,
            u32MaxPicWidth:1920,
            u32MaxPicHeight:1080,
            u32BufSize:(((1920.0 * 1080.0 * 1.5) as u32 + 4095) & (!4095)) *2,
            u32Profile:0,
            bByFrame:1,
            u32PicWidth:1920,
            u32PicHeight:1080,
            bSingleCore:0,
            bEsBufQueueEn:cvi_venc::CVI_H26X_ES_BUFFER_QUEUE_DEFAULT as u8,
            bIsoSendFrmEn:cvi_venc::CVI_H26X_ISO_SEND_FRAME_DEFAUL as u8,
            __bindgen_anon_1:cvi_venc::_VENC_ATTR_S__bindgen_ty_1{
                stAttrH265e:cvi_venc::VENC_ATTR_H265_S{
                    bRcnRefShareBuf:0,
                }
            },
        },
        stRcAttr:cvi_venc::VENC_RC_ATTR_S{
            enRcMode: cvi_venc::_VENC_RC_MODE_E_VENC_RC_MODE_H265FIXQP,
            __bindgen_anon_1:cvi_venc::_VENC_RC_ATTR_S__bindgen_ty_1{
                stH265FixQp:cvi_venc::VENC_H265_FIXQP_S{
                    u32Gop:cvi_venc::CVI_H26X_GOP_DEFAULT,
                    u32SrcFrameRate:30,
                    fr32DstFrameRate:30,
                    u32IQp:32,
                    u32PQp:32,
                    u32BQp:0,
                    bVariFpsEn:0,
                }
            },
        },
        stGopAttr:cvi_venc::VENC_GOP_ATTR_S{
            enGopMode:0,
            __bindgen_anon_1:cvi_venc::_VENC_GOP_ATTR_S__bindgen_ty_1{
                stNormalP:cvi_venc::VENC_GOP_NORMALP_S{
                    s32IPQpDelta:0,
                }
            },
        },
    };

    err_check!(cvi_venc::CVI_VENC_CreateChn(0,&venc_attr as *const  cvi_venc::VENC_CHN_ATTR_S));

    let src_chn = cvi_sys::MMF_CHN_S{
        enModId: cvi_sys::_MOD_ID_E_CVI_ID_VI,
        s32DevId: 0,
        s32ChnId: 0,
    };

    let dst_chn = cvi_sys::MMF_CHN_S{
        enModId: cvi_sys::_MOD_ID_E_CVI_ID_VENC,
        s32DevId: 0,
        s32ChnId: 0,
    };

    err_check!(cvi_sys::CVI_SYS_Bind(&src_chn as *const cvi_sys::MMF_CHN_S, &dst_chn as *const cvi_sys::MMF_CHN_S));


}

unsafe fn vi_venc_deinit(){

    let src_chn = cvi_sys::MMF_CHN_S{
        enModId: cvi_sys::_MOD_ID_E_CVI_ID_VI,
        s32DevId: 0,
        s32ChnId: 0,
    };

    let dst_chn = cvi_sys::MMF_CHN_S{
        enModId: cvi_sys::_MOD_ID_E_CVI_ID_VENC,
        s32DevId: 0,
        s32ChnId: 0,
    };
    
    err_check!(cvi_sys::CVI_SYS_UnBind(&src_chn as *const cvi_sys::MMF_CHN_S, &dst_chn as *const cvi_sys::MMF_CHN_S));

    err_check!(cvi_venc::CVI_VENC_StopRecvFrame(0));
    err_check!(cvi_venc::CVI_VENC_ResetChn(0));
    err_check!(cvi_venc::CVI_VENC_DestroyChn(0));


    

    err_check!(cvi_isp::CVI_ISP_Exit(0));
    stSnsGc2083_Obj.pfnUnRegisterCallback.unwrap()(0, &mut AE_LIB.clone() as *mut ALG_LIB_S, &mut AWB_LIB.clone() as *mut ALG_LIB_S);
    err_check!(cvi_ae::CVI_AE_UnRegister(0, &mut AE_LIB.clone() as *mut ALG_LIB_S as *mut cvi_ae::_ALG_LIB_S));
    err_check!(cvi_awb::CVI_AWB_UnRegister(0, &mut AWB_LIB.clone() as *mut ALG_LIB_S as *mut cvi_awb::_ALG_LIB_S));

    err_check!(cvi_vi::CVI_VI_DisableChn(0,0));

    err_check!(cvi_vi::CVI_VI_StopPipe(0));

    err_check!(cvi_vi::CVI_VI_DestroyPipe(0));

    err_check!(cvi_sys::CVI_SYS_Exit());
    err_check!(cvi_vb::CVI_VB_Exit());
}


fn create_mp4_file(filename:&str)->std::fs::File{
    let f = std::fs::File::options().create(true).write(true).truncate(true).open(filename).unwrap();
    f
}

static STOP:AtomicBool = AtomicBool::new(false);
static START_RECV_FRAME:AtomicBool = AtomicBool::new(false);


#[derive(Parser)]
#[command(author, version, about, long_about = None, arg_required_else_help(true))]
struct Cli{

    #[arg(short,long)]
    filename:String,

    #[arg(short,long)]
    dev:String,
}


fn main() {
    ctrlc::set_handler(||{
        STOP.store(true, std::sync::atomic::Ordering::Release);
        println!("prepare exit!");
    }).unwrap();

    let cli = Cli::parse();
    
    sensor::sensor_init(&cli.dev, &(cli.filename.clone() + ".gcsv"));

    unsafe {
        vi_venc_init();
        let mut venc_chn_status = cvi_venc::VENC_CHN_STATUS_S{
            u32LeftPics: 0,
            u32LeftStreamBytes: 0,
            u32LeftStreamFrames: 0,
            u32CurPacks: 0,
            u32LeftRecvPics: 0,
            u32LeftEncPics: 0,
            bJpegSnapEnd: 0,
            stVencStrmInfo: cvi_venc::VENC_STREAM_INFO_S{
                enRefType: 0,
                u32PicBytesNum: 0,
                u32PicCnt: 0,
                u32StartQp: 0,
                u32MeanQp: 0,
                bPSkip: 0,
                u32ResidualBitNum: 0,
                u32HeadBitNum: 0,
                u32MadiVal: 0,
                u32MadpVal: 0,
                u32MseSum: 0,
                u32MseLcuCnt: 0,
                dPSNRVal: 0.0,
            },
        };


        let mut f = create_mp4_file(&(cli.filename.clone() + ".mp4"));

        let recv_param = cvi_venc::VENC_RECV_PIC_PARAM_S{
            s32RecvPicNum:-1,
        };
    
        err_check!(cvi_venc::CVI_VENC_StartRecvFrame(0, &recv_param as *const cvi_venc::VENC_RECV_PIC_PARAM_S));
        START_RECV_FRAME.store(true, std::sync::atomic::Ordering::SeqCst);
        loop{
            err_check!(cvi_venc::CVI_VENC_QueryStatus(0, &mut venc_chn_status as *mut cvi_venc::VENC_CHN_STATUS_S));
            if venc_chn_status.u32CurPacks != 0{
                break;
            }
        }

        let mut stream = cvi_venc::VENC_STREAM_S{
            pstPack: null_mut(),
            u32PackCount: 0,
            u32Seq: 0,
            __bindgen_anon_1: cvi_venc::_VENC_STREAM_S__bindgen_ty_1{
                stH265Info:cvi_venc::VENC_STREAM_INFO_H265_S{
                    u32PicBytesNum: 0,
                    u32Inter64x64CuNum: 0,
                    u32Inter32x32CuNum: 0,
                    u32Inter16x16CuNum: 0,
                    u32Inter8x8CuNum: 0,
                    u32Intra32x32CuNum: 0,
                    u32Intra16x16CuNum: 0,
                    u32Intra8x8CuNum: 0,
                    u32Intra4x4CuNum: 0,
                    enRefType: 0,
                    u32UpdateAttrCnt: 0,
                    u32StartQp: 0,
                    u32MeanQp: 0,
                    bPSkip: 0,
                }
            },
            __bindgen_anon_2: cvi_venc::_VENC_STREAM_S__bindgen_ty_2{
                stAdvanceH265Info:cvi_venc::VENC_STREAM_ADVANCE_INFO_H265_S{
                    u32ResidualBitNum: 0,
                    u32HeadBitNum: 0,
                    u32MadiVal: 0,
                    u32MadpVal: 0,
                    dPSNRVal: 0.0,
                    u32MseLcuCnt: 0,
                    u32MseSum: 0,
                    stSSEInfo: [cvi_venc::VENC_SSE_INFO_S{
                        bSSEEn: 0,
                        u32SSEVal: 0,
                    };8],
                    u32QpHstgrm: [0;52],
                    u32MoveScene32x32Num: 0,
                    u32MoveSceneBits: 0,
                }
            },
        };
        
        let mut area:ManuallyDrop<Vec<cvi_venc::VENC_PACK_S>> =  ManuallyDrop::new(Vec::with_capacity(venc_chn_status.u32CurPacks as usize));

        stream.pstPack = area.as_mut_ptr();


        println!("curPacks:{}",venc_chn_status.u32CurPacks);
        let mut cnt = 0;
        loop{
            // if cvi_venc::CVI_VENC_GetStream(0, &mut stream as *mut cvi_venc::VENC_STREAM_S, 2000) !=0 || cnt >=100{
            //     break;
            // }
            if cvi_venc::CVI_VENC_GetStream(0, &mut stream as *mut cvi_venc::VENC_STREAM_S, 2000) !=0  
                || STOP.load(std::sync::atomic::Ordering::SeqCst){
                break;
            }
            //println!("curPackCount:{} cnt:{}",stream.u32PackCount,cnt);
            for j in 0..stream.u32PackCount{
                let pack  =  *(stream.pstPack.wrapping_add(j as usize));
                f.write(std::slice::from_raw_parts(pack.pu8Addr.wrapping_add(pack.u32Offset as usize), 
                  (pack.u32Len - pack.u32Offset) as usize)).unwrap();
                cnt +=1;
            }
            err_check!(cvi_venc::CVI_VENC_ReleaseStream(0,&mut stream as *mut cvi_venc::VENC_STREAM_S));

        }
        f.flush().unwrap();
        std::mem::ManuallyDrop::drop(&mut area);

        vi_venc_deinit();

    }
    println!("Hello, world!");

}
