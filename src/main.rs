mod cvi_isp;
mod cvi_sns_ctrl;
mod cvi_sys;
mod cvi_vb;
mod cvi_vi;
mod cvi_comm_vi;
mod cvi_ae;
mod cvi_awb;

use cvi_vb::VB_POOL_CONFIG_S;
use std::os::raw::c_void;

use crate::{
    cvi_isp::{CVI_MIPI_SetClkEdge, CVI_MIPI_SetMipiReset, CVI_MIPI_SetSensorReset, CVI_MIPI_SetMipiAttr, CVI_VOID, CVI_MIPI_SetSensorClock, CVI_ISP_SetBindAttr, CVI_ISP_MemInit, ISP_PUB_ATTR_S, },
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
    },
    cvi_comm_vi::{}, cvi_ae::CVI_AE_Register, cvi_awb::CVI_AWB_Register
};

macro_rules! err_check {  
    ($expr:expr) => { 
        let _tmp = $expr;
        if(_tmp != 0){
            println!("error at {}, return:{:x}",line!(), _tmp);
        } 
    };  
}

#[no_mangle] 
pub extern "C" fn __sync_fetch_and_and_1(p:*mut c_void, v:u32) -> u32{
    unsafe{
        std::sync::atomic::AtomicU32::from_ptr(p as *mut u32).fetch_and(v, std::sync::atomic::Ordering::Relaxed)
    }
}


fn main() {
    unsafe {
        let mut temp = cvi_sys::MMF_VERSION_S { version: [0; 128] };

        cvi_sys::CVI_SYS_GetVersion(&mut temp as *mut cvi_sys::MMF_VERSION_S);

        CVI_VI_SetDevNum(1);

        CVI_SYS_Exit();
        CVI_VB_Exit();

        let vb_config = VB_CONFIG_S {
            u32MaxPoolCnt: 1,
            astCommPool: [VB_POOL_CONFIG_S {
                u32BlkSize: ((1920 * 10 + 7) / 8 * 1080 + 0x3F) & (!0x3F),
                u32BlkCnt: 4,
                enRemapMode: 2, // 2 is VB_REMAP_MODE_CACHED
                acName: [0; 32],
            }; 16],
        };
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

        let mut ae_lib_name = [0 as u8;20];
        let mut awb_lib_name = [0 as u8;20];
        std::ptr::copy_nonoverlapping("cvi_ae_lib".as_ptr(), ae_lib_name.as_mut_ptr() as *mut u8, "cvi_ae_lib".len());
        std::ptr::copy_nonoverlapping("cvi_awb_lib".as_ptr(), awb_lib_name.as_mut_ptr() as *mut u8, "cvi_awb_lib".len());
        let mut ae_lib = ALG_LIB_S{
            s32Id: 0,
            acLibName: ae_lib_name
        };

        let mut awb_lib = ALG_LIB_S{
            s32Id: 0,
            acLibName: awb_lib_name
        };
        
        stSnsGc2083_Obj.pfnRegisterCallback.unwrap()(0, &mut ae_lib as *mut ALG_LIB_S, &mut awb_lib as *mut ALG_LIB_S);

        let mut sns_dev_attr: combo_dev_attr_s = SNS_COMBO_DEV_ATTR_S {
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
            enIntfMode: 5, // VI_MODE_MIPI
            enWorkMode: 0, // VI_WORK_MODE_1Multiplex
            enScanMode: 1, // VI_SCAN_PROGRESSIVE
            as32AdChnId: [-1; 4],
            enDataSeq: 4, // VI_DATA_SEQ_YUYV
            stSynCfg: VI_SYNC_CFG_S {
                enVsync: 1,         // VI_VSYNC_PULSE
                enVsyncNeg: 1,      // VI_VSYNC_NEG_LOW
                enHsync: 0,         // VI_HSYNC_VALID_SINGNAL
                enHsyncNeg: 0,      // VI_HSYNC_NEG_HIGH
                enVsyncValid: 1,    // VI_VSYNC_VALID_SIGNAL
                enVsyncValidNeg: 0, // VI_VSYNC_VALID_NEG_HIGH
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
            enInputDataType: 1, // VI_DATA_TYPE_RGB
            stSize: _SIZE_S {
                u32Width: 1920,
                u32Height: 1080,
            },
            stWDRAttr: VI_WDR_ATTR_S {
                enWDRMode: 0,
                u32CacheLine: 1080,
                bSyntheticWDR: 0,
            },
            enBayerFormat: 3, // BAYER_FORMAT_RG
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
            enCompressMode: 0,
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

        err_check!(CVI_AE_Register(0, &mut ae_lib as *mut ALG_LIB_S as *mut cvi_ae::_ALG_LIB_S));
        err_check!(CVI_AWB_Register(0, &mut awb_lib as *mut ALG_LIB_S as *mut cvi_awb::_ALG_LIB_S));

        let isp_bind_attr = ISP_BIND_ATTR_S{
            sensorId: 0,
            stAeLib: ae_lib,
            stAfLib: ALG_LIB_S{
                s32Id: 0,
                acLibName: [0;20],
            },
            stAwbLib: awb_lib,
        };

        CVI_ISP_SetBindAttr(0, &isp_bind_attr as *const ISP_BIND_ATTR_S as *const cvi_isp::_ISP_BIND_ATTR_S);

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

        std::ptr::write_bytes(isp_static_cfg.stAECfg.au8Weight.as_mut_ptr(), 1, size_of_val(&isp_static_cfg.stAECfg.au8Weight));

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

        err_check!(cvi_isp::CVI_ISP_Run(0));
    }
    println!("Hello, world!");
}
