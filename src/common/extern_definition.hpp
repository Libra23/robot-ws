#ifndef EXTERN_DEFINITION_H
#define EXTERN_DEFINITION_H

#include "memory.hpp"
#include "queue.hpp"
#include "control_data/io_data.hpp"

/**
 * @brief extern memory defined at io_interface
 */
extern ShareMemory<OutputState> output_memory_;
extern ShareMemory<InputState> input_memory_;

/**
 * @brief extern queue defined at mainte
 */
extern Queue mainte_to_robot_queue_;

#endif