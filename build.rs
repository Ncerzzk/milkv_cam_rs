fn main() {  
    // 如果你的静态库是由C源代码编译的，你可以这样编译它：  
    // cc::Build::new()  
    //     .file("path/to/your/c_source_file.c")  
    //     .compile("mylib"); // 这将生成 libmylib.a（在Unix-like系统上）  
 
    // 由于我们假设静态库已经存在，我们只需要设置链接器标志

    println!("cargo:rustc-link-search=native=/home/ncer/duo-buildroot-sdk/middleware/v2/lib/");    
    println!("cargo:rustc-link-lib=static=sys");
    println!("cargo:rustc-link-lib=static=sns_gc2083");
    println!("cargo:rustc-link-lib=static=vpu");
    println!("cargo:rustc-link-lib=static=isp");
    println!("cargo:rustc-link-lib=static=ae");
    println!("cargo:rustc-link-lib=static=awb");
    println!("cargo:rustc-link-lib=static=af");
    println!("cargo:rustc-link-lib=static=isp_algo");

    println!("successfully set link flag!");
    // 如果静态库不在标准库路径中，你需要指定库文件的路径  

 
    // 如果你有头文件需要包含，并且它们不在标准包含路径中，  
    // 你可以使用 cc crate 的 .include() 方法来添加包含路径（但这通常用于编译C代码时）  
    // cc::Build::new()  
    //     .include("path/to/your/include")  
    //     // ... 其他配置 ...  
    //     .finish(); // 注意：这里我们实际上没有调用.compile()，因为我们只是设置包含路径  
 
    // 注意：在这个例子中，我们没有使用 cc crate 来编译 C 代码，  
    // 只是用它可能的方法来展示如何设置包含路径（尽管在这个上下文中不是必需的）。  
    // 对于实际的链接，我们只需要上面的 println! 宏调用。  
}