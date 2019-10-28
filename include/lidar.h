#ifndef _LIDAR_H_
#define _LIDAR_H_

/**
 * This API was made from scratch using the specifications of the RPLidar
 * found at http://www.slamtec.com/en/support#rplidar-a2
 * It includes all functions of the RPLidar except the Express scan, which
 * would have been too computationally taxing to decode compared to the
 * traditional scan, resulting in a slower response time.
 * Notes:
 *   -Start the lidar with initLidar()
 *   -lidarScan() retrieves and decodes a single 360 degree scan
 *   -Always use lidarShutdown() before turning off the system
 */


/**
 * Initializes uart communication with lidar and starts lidar motor.
 * Will attempt to reset lidar if hardware issue exists.
 *
 * @return 0 on success, negative integer for communication error,
 * and positive integer for hardware error
 */
int initLidar();

/**
 * Initiates scan procedure and decode process.
 * Only completes a single 360 degree scan.
 */
void lidarScan();


void lidarThreadedScan(void *);

/**
 * Returns reference to the decoded distance array.
 * Index refers to the angle at which the distance element was found.
 * If using a seperate thread for scanning, the array will fully update every 0.5 seconds.
 * Do not modify this array!
 */
int * lidarGetDistances();

 /**
  * Retrieves lidar device information and prints it to the terminal
  */
void lidarGetInfo();

/**
 * Prints the health state of the lidar and error code is there exists a problem
 * with the unit
 * @return lidar health status
 */
int lidarGetHealth();

/**
 * Displays the samplerate (time taken to complete a full scan) for standard
 * and express scans
 */
void lidarGetSampleRate();

/**
 * Sends the scan request (and nothing else)
 */
void lidarStartScan();

/**
 * Stops the lidar from scanning
 */
void lidarStopScan();

/**
 * Resets the lidar
 */
void lidarReset();

/**
 * Stop scannning, shutdown usart communication, and stop lidar motor.
 * Must use initLidar() to start lidar again.
 */
void lidarShutdown();

/**
 * Prints decoded angles and distances from most recent scan for debugging
 */
void lidarPrintDistances();

/**
 * Flush the lidar buffer. Helpful when interrupting a scan to perform other
 * lidar requests
 */
void lidarFlushBuff();

/**
 * If wireless transeiver module attached, carelessly send distance array
 * using uart2 port
 */
void wirelessSend();



#endif
