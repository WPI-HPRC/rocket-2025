//
// Created by Daniel Coburn on 11/16/24.
//
#pragma once
#include "Task.h"
#include <vector>


class TaskScheduler {
public:
    void run();
    bool add(Task* task);
    void remove(Task* task);

private:
    std::vector<Task*> taskList;

protected:
};
