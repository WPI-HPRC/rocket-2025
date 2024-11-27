//
// Created by Daniel Coburn on 11/16/24.
//

#include "TaskScheduler/TaskScheduler.h"
#include <Arduino.h>

// init with empty list
TaskScheduler::TaskScheduler() : taskList() {}

/**
 * public \n
 * runs all tasks in the TaskScheduler
 */
void TaskScheduler::run() {
    for(Task* currentTask: taskList) {
        currentTask->run();
    }
}
/**
 * public \n
 * adds a function pointer to the TaskScheduler
 * @param task pointer to the function that is the task
 * @return true  if task was added, false  if not
 */
bool TaskScheduler::add(Task *task) {
    auto it = std::find(taskList.begin(), taskList.end(), task);

    if (it == taskList.end()) {
        taskList.push_back(task);
        return true;
    }
    return false;
}

/**
 * public \n
 * removes a function pointer form the TaskScheduler
 * @param task pointer to the function to remove
 * @retun true if task was removed, false if not
 */
bool TaskScheduler::remove(Task *task) {
    auto it = std::find(taskList.begin(), taskList.end(), task);

    if(it != taskList.end()) {
        taskList.erase(it, taskList.end());
        return true;
    }
    return false;
}
