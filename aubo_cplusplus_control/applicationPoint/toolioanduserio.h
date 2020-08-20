#ifndef TOOLIOANDUSERIO_H
#define TOOLIOANDUSERIO_H

class ToolioAndUserIO
{
public:
    ToolioAndUserIO();


public:
    /**
     * @brief run
     *
     * 应用案例：通过工具端IO接按钮实现拖动示教
     *
     *
     * 原理：
     * 　　1. 获取工具端IO指定DI的状态, 进行逻辑判断然后设置用户IO指定DO的状态
     *       如果工具端DI有效　　设置指定用户DO有效
     * 　　　　如果工具端DI无效　　设置指定用户DO无效
     *
     *    2.  用户IO接继电器　 常开触点接SI06和0V　　控制SI06与0V的通断
     */
    static void run();

};

#endif // TOOLIOANDUSERIO_H
