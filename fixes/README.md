# SDK 本地存储溢出修正

适用于原交付源码在SDK链接时出现 `.text will not fit` / `microblaze_0_local_memory overflowed` 的情况。原工程本地BRAM为64KiB，原应用使用的标准库格式化/解析可能使目标程序超出此容量。

本目录的 `helloworld.c` 取消了 snprintf、strtol、strtoul，改为有边界检查的整数格式化与解析。WiFi帧格式不变，不需要修改App或PL。输入整数执行严格十进制检查，seq限制为1～4294967295。原交付ZIP和三份说明文件保持不变。

操作：
1. 下载本目录的helloworld.c。
2. 在SDK中用其完整内容替换 `home6/src/helloworld.c`，保存。只保留一个应用入口main。
3. 右键home6 → Properties → C/C++ Build → Settings → MicroBlaze gcc compiler → Optimization，将Optimization Level设置为Optimize for size (-Os)，Apply and Close。不同SDK版本的选项名可能略有区别。
4. Project → Clean，选择home6；完成后Project → Build All。
5. 确认Debug/home6.elf生成且没有Error，再进行下载。

不要只改lscript.ld中的LENGTH来假装扩容：实际BRAM仍是64KiB。若修正后仍溢出，请提供Build Console中完整链接错误（尤其是overflowed by多少bytes）、lscript.ld的MEMORY段、编译命令与map文件，再判断是否需要扩大真实BRAM。

已验证：主机GCC在-Wall -Wextra -Werror下通过；原风扇/日期/拆包粘包测试通过；新增状态帧逐字符一致、ACK最大序号、整数溢出拒绝、短缓冲区边界测试通过；对象文件没有printf/scanf/strtol系列依赖。未在MicroBlaze交叉工具链上链接，不能保证具体目标代码尺寸；最终以SDK构建结果为准。
