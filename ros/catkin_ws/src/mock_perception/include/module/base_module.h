// 模块基类

#pragma once

#include <iostream>

class BaseModule
{
public:
    BaseModule() = default;
    virtual ~BaseModule();

    virtual bool run() = 0;
};