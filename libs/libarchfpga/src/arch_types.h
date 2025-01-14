/*
 * Data types describing the FPGA architecture.
 *
 * Date: February 19, 2009
 * Authors: Jason Luu and Kenneth Kent
 */

#ifndef ARCH_TYPES_H
#define ARCH_TYPES_H

#include "logic_types.h"
#include "physical_types.h"
#include "cad_types.h"

/* Input file parsing. */
#define TOKENS " \t\n"

/* Value for UNDEFINED data */
constexpr int UNDEFINED = -1;

/** The total number of predefined blif models */
constexpr int NUM_MODELS_IN_LIBRARY = 4;

/* Maximum value for minimum channel width to avoid overflows of short data type.               */
constexpr int MAX_CHANNEL_WIDTH = 8000;

/* Built-in library models */
constexpr const char* MODEL_NAMES = ".names";
constexpr const char* MODEL_LATCH = ".latch";
constexpr const char* MODEL_INPUT = ".input";
constexpr const char* MODEL_OUTPUT = ".output";

enum class e_arch_format {
    VTR,            ///<VTR-specific device XML format
    FPGAInterchange ///<FPGA Interchange device format
};

#endif
