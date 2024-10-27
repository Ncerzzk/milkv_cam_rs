duo_sdk_path=/home/ncer/duo-buildroot-sdk
include_path=$duo_sdk_path/middleware/v2/include
include_path2=$duo_sdk_path/middleware/v2/include/isp/cv180x
pushd src

function gen(){
    header=$1
    outputrs=$2
    bindgen $include_path/$header -o $outputrs    --no-doc-comments   -- -I$include_path -I$include_path2 -DARCH_CV180X
    sed -i '1i#![allow(dead_code, non_snake_case, non_camel_case_types, non_upper_case_globals, unused_imports)]'  $outputrs
}

#bindgen $include_path/cvi_vi.h -o cvi_vi.rs    --no-doc-comments   -- -I$include_path
#bindgen $include_path/cvi_venc.h -o cvi_venc.rs    --no-doc-comments   -- -I$include_path
#bindgen $include_path/cvi_sys.h -o cvi_sys.rs    --no-doc-comments   -- -I$include_path
gen cvi_vi.h cvi_vi.rs
gen cvi_venc.h cvi_venc.rs
gen cvi_sys.h cvi_sys.rs
gen cvi_sns_ctrl.h cvi_sns_ctrl.rs
gen cvi_vb.h cvi_vb.rs
gen isp/cv180x/cvi_isp.h cvi_isp.rs
gen isp/cv180x/cvi_ae.h cvi_ae.rs
gen isp/cv180x/cvi_awb.h cvi_awb.rs
gen linux/cvi_comm_vi.h cvi_comm_vi.rs
