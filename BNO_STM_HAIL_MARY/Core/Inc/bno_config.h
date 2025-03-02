/*
 * bno_config.h
 *
 *  Created on: Dec 23, 2023
 *      Author: wardawg
 */

#ifndef BNO_CONFIG_H_
#define BNO_CONFIG_H_

/**
 * Uncomment if you want to automatically set the Page to use for the Bosch
 * BNO055. Activating this will result in a larger binary and a small bit slower
 * execution of the Data read functions.
 * If you keep this uncommented, you can set the Page by `bno055_set_page(...)`.
 */
#define BNO_AUTO_PAGE_SET

/**
 * TODO: Add usage for smaller memory footprint
 */
// #define BNO_SMALL_MEMORY_FOOTPRINT

#endif /* BNO_CONFIG_H_ */
