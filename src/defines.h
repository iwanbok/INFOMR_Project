#pragma once

#ifndef ASSET_PATH
#define ASSET_PATH "data/"
#endif

// #define L2_METRIC // UNCOMMENT FOR L2 Distance metric
// #define EMD_METRIC // UNCOMMENT FOR EMD Distance metric (does not work in ANN, overides L2 in
// Custom metric)
#define ORIGINAL_DIR ASSET_PATH "LabeledDB_new"
#define PREPROCESSED_DIR ASSET_PATH "preprocessed"
#define NORMALIZED_DIR ASSET_PATH "normalized"
#define FEATURE_DIR ASSET_PATH "features"
#ifdef L2_METRIC
#define WEIGHTS_FILE ASSET_PATH "weights - L2.txt"
#else
#define WEIGHTS_FILE ASSET_PATH "weights - L1.txt"
#endif