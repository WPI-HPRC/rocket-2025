//
// Created by Daniel Coburn on 11/16/24.
//

#include "TaskScheduler/TaskScheduler.h"
#include <Arduino.h>

TaskScheduler::TaskScheduler() : taskList() {}

void TaskScheduler::run() {
    // run tasks
    for(Task* currentTask: taskList) {
        currentTask->run();
    }
}

bool TaskScheduler::add(Task *task) {
    auto it = std::find(taskList.begin(), taskList.end(), task);

    if (it == taskList.end()) {
        taskList.push_back(task);
        return true;
    }
    return false;
}

void TaskScheduler::remove(Task *task) {
    auto it = std::find(taskList.begin(), taskList.end(), task);

    if(it != taskList.end()) {
        taskList.erase(it, taskList.end());
        // i blindly copied this, have no real clue if its correct
    }
}
