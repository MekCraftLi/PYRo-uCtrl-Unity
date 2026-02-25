/**
 * @file pyro_module_base.tpp
 * @brief Implementation of the PYRO Module Base Template.
 * PYRO 模块基类模板实现文件。
 *
 * This file contains the implementation of the `pyro::module_base_t` template
 * class. It handles the low-level details of command double-buffering,
 * thread-safe mutex locking, and the internal task loop execution logic.
 * 本文件包含了 `pyro::module_base_t` 模板类的实现。它处理命令环形缓冲区
 * 线程安全互斥锁以及内部任务循环执行逻辑的底层细节。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-01-28
 */

#pragma once

#include "pyro_core_def.h"
namespace pyro
{

template <typename Derived, typename CmdType, typename ModuleDeps>
module_base_t<Derived, CmdType, ModuleDeps>::module_base_t(
    const char *name, uint16_t init_stack, uint16_t loop_stack,
    task_base_t::priority_t priority)
    : _task(this, name, init_stack, loop_stack, priority)
{
}

template <typename Derived, typename CmdType, typename ModuleDeps>
status_t module_base_t<Derived, CmdType, ModuleDeps>::module_task_t::init()
{
    if (_owner)
       return _owner->_init();
    return status_t::PYRO_ERROR;
}

/**
 * @brief Invokes the core loop implementation of the module instance.
 * 调用模块实例的核心循环实现。
 */
template <typename Derived, typename CmdType, typename ModuleDeps>
void module_base_t<Derived, CmdType, ModuleDeps>::module_task_t::run_loop()
{
    if (_owner)
        _owner->_run_loop_impl();
}

} // namespace pyro