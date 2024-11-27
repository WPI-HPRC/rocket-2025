//
// Created by Daniel Coburn on 11/16/24.
//
#pragma once
#include "Task.h"
#include <vector>
#include "services/Time.h"

class TaskScheduler {
public:
    TaskScheduler();
    void run();
    bool add(Task* task);
    bool remove(Task* task);

private:
    std::vector<Task*> taskList;
    Time* timer;

protected:
};
