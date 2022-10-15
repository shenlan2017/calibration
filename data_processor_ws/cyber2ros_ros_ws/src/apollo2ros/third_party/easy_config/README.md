# easy_config

[![Ubuntu](https://github.com/incloon/easy_config/actions/workflows/ubuntu-unit.yml/badge.svg?branch=main)](https://github.com/incloon/easy_config/actions/workflows/ubuntu-unit.yml)
[![Windows](https://github.com/incloon/easy_config/actions/workflows/windows-unit.yml/badge.svg?branch=main)](https://github.com/incloon/easy_config/actions/workflows/windows-unit.yml)
[![macOS](https://github.com/incloon/easy_config/actions/workflows/macos-unit.yml/badge.svg?branch=main)](https://github.com/incloon/easy_config/actions/workflows/macos-unit.yml)

[中文](README.md) | [English (TODO)](doc/README_English.md)

`easy_config` 是一个 `C++11` 运行期结构体聚合初始化反射库，可以允许您在运行期将聚合初始化的内容读入您自定义的结构体中，通过几步非常简单的设置即可将此项目嵌入您的项目并实现反射，此项目使用了 `CMake` 来生成用户的结构体聚合初始化解析代码，除了标准库外没有其他外部依赖

`easy_config` 目前尚处于早期阶段，可能会出现一些bug，但主要的功能已经在 `gcc` 和 `vs2019` 上进行了测试，如果发现任何BUG，请第一时间提 `issue`



## 简单示例

对于下面用户定义的结构体文件 `str.h`

```c++
#pragma once
#include <string>
#include <vector>
struct Str
{
    char c;
    int i;
    float f;
    std::string str;
    ::std::vector<int> v;
};
```

`easy_config` 可以将以下结构体的聚合初始化读入结构体

```c++
//Str str =
{
    .c='\t',
    .i = (3u + 3) / 2,
    .f = 3333.e-3,
    .str = "this" " is string",
    .v = {1, 2, 3,},
};//结尾的分号可以省略
```

假设上面的聚合初始化保存在文件 `ini.txt` 中，只需要以下代码就可完成反射

```c++
#include <str.h>
#include <interpreter.hpp>

int main()
{
    ezcfg::Interpreter itp("ini.txt");
    Str str;
    itp.parse(str);
}
```

`easy_config` 要求您将需要反射的结构体定义放在单独的文件内，并且要求结构体不能继承其他结构体且内部仅有成员变量声明，您可以将需要反射的结构体全部置于同一文件内，也可以分为多个文件，文件中预处理指令将全部被忽略



## 快速配置

推荐您使用 `CMake` 来简化本项目的配置过程，但 `CMake` 不是必须的，如果要使用其他方式配置本项目，您可能需要自己生成结构体解析代码并将其加入构建

1. 下载或克隆 `easy_config` 至您的工程下，为便于说明，假设需要反射的结构体文件位置为 `src/struct.h`，`easy_config` 位于目录 `third/easy_config`中

2. 在您项目的顶层 `CMakeLists.txt` 中加入如下内容
   
   ```cmake
   set(EZCFG_STRUCT_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/struct.h) #必须为绝对路径，如有多个文件使用空格隔开
   add_subdirectory(third/easy_config)
   target_link_libraries(your_target PRIVATE ezcfg::ezcfg)
   ```

现在你已经完成了全部的配置，现在你可以按照简单示例中的用法执行反射了XD



## API

`Interpreter` 的主要接口已经在下方代码块列出

```c++
class Interpreter
{
public:
	Interpreter();
    
	//is_file的值决定了将str解释为文件名还是解析文本（1）
	Interpreter(const std::string& str, bool is_file = true);
    
	//指定新的文件作为解析文本，当前解析文本将被抛弃（2）
	bool loadFile(const std::string& file);
    
	//将一个新的字符串作为解析文本，当前解析文本将被抛弃（3）
	bool loadSource(const std::string& source);
    
	//批量数据解析接口（4）
	template<typename T, typename... TS>
	inline void parse(T& data, TS&... datas);
    
	//专门解析表达式的接口，其返回类型可以转换为任何C++内置数值类型（5）
	inline ArithmeticT parseExpression();

	//指示是否已经到了文件或字符串结尾
	explicit operator bool() const;
}
```

（1）（2）（3）是与构造加载相关的功能，使用方法参考以下代码块

```c++
ezcfg::Interpreter itp1("demo.txt");//构造时加载文件demo.txt做为解析文本
ezcfg::Interpreter itp2("{1, 2, 3}", false);//构造时将字符串"{1, 2, 3}"做为解析文本
itp1.loadSource("{4, 5, 6}");//加载"{4, 5, 6}"做为解析文本，构造时加载的文件demo.txt被抛弃
itp1.loadFile("demo.txt");//加载文件demo.txt做为解析文本，构造时加载的字符串"{1, 2, 3}"被抛弃
```

（4）（5）是解析相关的接口，（4）可以批量解析任意多的参数，但是这些参数的顺序必须与解析文本中的顺序相同，（5）专门用来解析表达式，表达式不能含有花括号，他们的用法和区别参考以下代码块

```c++
std::vector<int> int_vec;
std::vector<double> double_vec;
ezcfg::Interpreter("{1, 2, 3};" "{4., 5., 6.};", false).parse(int_vec, double_vec);
//执行后int_vec的值为{1, 2, 3}，double_vec的值为{4., 5., 6.}

//以上解析也可以分为两次进行
ezcfg::Interpreter itp("{1, 2, 3};" "{4., 5., 6.};", false);
itp.parse(int_vec);//执行后int_vec的值为{1, 2, 3}
itp.parse(double_vec);//执行后double_vec的值为{4., 5., 6.}

//parseExpression用法如下
int a = ezcfg::Interpreter("1+1;", false).parseExpression();//此时a的值为2

//parse和parseExpression的区别如下
ezcfg::Interpreter("{1};", false).parse(a);//允许，此时a的值为1
ezcfg::Interpreter("{1};", false).parseExpression();//不允许，此时解析遇到错误，导致程序退出
//以上区别也符合C++的对于表达式的定义，parseExpression功能仅为计算表达式的值，而parse是将初始化文本进行反射
```



## 实现原理

`easy_config` 由一个公用的词法分析器、一个生成自定义结构体初始化解析代码的编译器和一个反射聚合初始化的解释器构成，当用户将 `easy_config` 设置为`CMake` 中的子项目时，`easy_config` 会先编译生成编译器部分，之后使用生成的编译器编译用户的结构体代码，生成解析用户定义的结构体代码，再将生成的代码与用户代码链接从而实现反射，生成的代码是解析对应结构体函数模板的特化的实现。

如果要脱离 `CMake` 使用 `easy_config` 则首先需要编译 `src` 文件下的 `compiler.cpp` 得到可执行文件 `compiler` 之后使用如下命令得到解析用户定义的结构体的代码

> compiler [input_file_path...] [output_file]

之后包含 `easy_config` 的所有头文件并将输出文件与用户代码一起编译即可完成反射

`easy_config` 的解释器使用了元编程技术，解析时将依次对结构体成员进行遍历并将使用与成员类型匹配的解析函数解析此成员，因此如果没有匹配的函数时编译将出错。每个解析函数都是在对结构体成员使用于之匹配的解析函数，因此这些解析函数之间可以嵌套，所以 `easy_config` 可以解析由自定义结构体组成的容器或是不同自定义结构体之间的嵌套



## 语法支持

`easy_config` 希望与C++20标准保持一致，但由于C++标准的复杂性。所以目前实现的功能仅为标准相关部分的一个子集，但是目前实现的语法已经能够覆盖绝大多数使用场合。`easy_config` 完整实现了标准规定的布尔、整数、浮点数文法以及除与Unicode有关的全部字符、字符串文法，并且支持注释和“\“换行，整个处理过程完全遵守标准的规定，支持字面量之间的`+`、`-`、`*`、`/`、`%`、`,` 、`()`运算，运算结果与编译器计算结果一致，支持所有标准容器

`easy_config` 也存在少量的特殊的规则，这些规则设计的目的是为了安全性考虑，主要有如下几点

1. 自定义结构体的聚合初始化不允许省略成员名称
2. 不允许跳过自定义结构体的成员，除非此成员具有默认值
3. 数组必须被完整填充，多维数组不允许省略花括号，如果数组类型为 `char` `signed char` `unsigned char` 并且使用字符串文法赋值则不需要完整填充， `std::array` 被视为数组

因为上述规则，所以建议您谨慎使用数组作为数据类型，如果数据长度有可能发生变化则建议使用 `std::vector` 来代替数组的使用

