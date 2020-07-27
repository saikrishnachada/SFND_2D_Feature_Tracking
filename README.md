# **Project : Camera Based 2D Feature Tracking** 

To build a camera based 2D feature tracking for a collision avoidance system from the sequence of mono camera images mounted on top of a vehicle and to test various detector, descriptor combinations to see which ones perform best.

#### Objectives 
   
(1)	Setting up data structures and loading images and put everything into a ring buffer to optimize the memory load  
(2)	Integrate several keypoint detectors such as Harris, FAST, BRISK and also SIFT and compare them to each other with regard to the number of key points and with regard to speed   
(3)	Next we focus on descriptor extraction and matching using brute-force and also FLANN approach  
(4)	Test the various algorithms and compare them with regard to some performance measures  

[//]: # (Video References)

[video1]: ./Run_video_finalparameters.mkv "A run using final parameters"
---

#### MP.1 Data Buffer Optimization
To tackle the growing size of the data structure, the dataBuffer has been modified from std::vector to std::deque. It is a double-ended queue and is an indexed sequence container that allows fast insertion and deletion at both its beginning and its end. 
```
 #include <deque>
 deque<DataFrame> dataBuffer;
```
---
#### MP.2 Keypoint Detection
A list of keypoint detectors (SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT) have been implemented from lines 79-85 in MidTermProject_Camera_Student.cpp. They can be selected by uncommenting respective strings. 

```
   string detectorType = "SHITOMASI";
// string detectorType = "HARRIS";
// string detectorType = "FAST";
// string detectorType = "BRISK";
// string detectorType = "ORB";
// string detectorType = "AKAZE";
// string detectorType = "SIFT";
```
---
#### MP.3 Keypoint Removal
The main focus of this project in this course is on collision detection system and the keypoints on the preceding vehicle are of special interest to us. Therefore, all the keypoints outside of a pre-defined rectangle are removed and only the keypoints within this rectangle bounding are used for further processing. The code is implemented in the lines 127-136 in MidTermProject_Camera_Student.cpp.
```
        // Filter with a hardcoded bounding box to keep only keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> filteredKeypoints;
            for (auto kp : keypoints) {
                if (vehicleRect.contains(kp.pt)) filteredKeypoints.push_back(kp);
            }
            keypoints = filteredKeypoints;
        }
```
---
#### MP.4 Keypoint Descriptors
A list of keypoint descriptors (BRISK, BRIEF, ORB, FREAK, AKAZE and SIFT) have been implemented from lines 172-177 in MidTermProject_Camera_Student.cpp. They can be selected by uncommenting respective strings. 
```
   string descriptorType = "BRISK";
// string descriptorType = "BRIEF";
// string descriptorType = "ORB";
// string descriptorType = "FREAK";
// string descriptorType = "AKAZE";  
// string descriptorType = "SIFT";
```
---
#### MP.5 Descriptor Matching
Implemented FLANN matching as well as k-nearest neighbor selection from lines 193-210 in MidTermProject_Camera_Student.cpp. Additionally, the function matchDescriptors in matching2D_Student.cpp has been written to contain all the descriptor categories (DES_BINARY, DES_HOG), matcher types (MAT_FLANN, MAT_BF) and selector types (SEL_NN, SEL_KNN). 

---
#### MP.6 Descriptor Distance Ratio
Used the K-Nearest-Neighbor matching to implement the descriptor distance ratio test in matching2D_Student.cpp from lines 76-82, which looks at the descriptor distance ratio test and filters the false-positive keypoint matches. 
```
double minDescDistRatio = 0.8;
for (auto it : knn_matches) {
 // The returned knn_matches vector contains some nested vectors with size < 2 !?
    if ( 2 == it.size() && (it[0].distance < minDescDistRatio * it[1].distance) ) {
        matches.push_back(it[0]);
    }
 }
```
---
####  MP.7 Performance Evaluation 1
The follwoing table shows the number of keypoints for all the descriptors on the preceding vehicle and respective neighborhood sizes for all 10 images.

|  Images      | SHITOMASI <br>Kps,Nsize               | HARRIS <br>Kps,Nsize                   |FAST <br>Kps,Nsize  | BRISK <br>Kps,Nsize               | ORB <br>Kps,Nsize                   |AKAZE <br>Kps,Nsize |SIFT <br>Kps,Nsize |
| ------------- | ----------------------- | ----------------------- |----------------------- | ----------------------- | ----------------------- |----------------------- |----------------------- |
| 1      | 93 & 5 | 17 & 6|419  & 7 |264 & 11.444 | 92	& 31 |166 &	4.8 |138	& 2.013 |
| 2      | 86 & 5 | 14 & 6|427 & 7 | 282 & 16.548 | 102 &	31 |157 &	4.8  |132 &	4.969 |
| 3      | 99 & 5 |  19 & 6|404 & 7 | 282 & 13.073 | 106 &	31 |161 &	4.8  |124	& 5.103 |
| 4      | 90 & 5  |  22 & 6|423 & 7| 277 & 11.600 | 113 & 31 |155 &	4.8  |138	& 2.999 |
| 5      | 88 & 5  |  26 & 6|386 & 7 | 297 & 12.000 | 109 &	31 |163 &	4.8  |134 &	3.312 |
| 6      | 96 & 5  | 47 & 6|414 & 7 | 279 & 12.183 | 125 & 31 |164 &	4.8  |140	& 2.025 |
| 7      | 87 & 5  | 18 & 6|418 & 7 | 289 & 11.545 | 130 &	31 |173 &	4.8  |137 &	3.032 |
| 8      | 98 & 5  | 33 & 6|406 & 7 | 272 & 13.620 | 129 &	31 |175 &	4.8  |148	& 1.866 |
| 9      | 98 & 5  | 27 & 6|396 & 7 | 266 & 10.571 | 127 &	31|177 &	4.8  |159	& 2.096 |
| 10      | 98 & 5 | 35 & 6|401 & 7 | 254 &	8.400 | 128 &	31 |179 &	4.8  |137	& 2.089|
---
#### MP.8 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8. neighborhood sizes for all 10 images.
|  Images      | SHITOMASI + BRISK <br> Kps & Time               | SHITOMASI + BRIEF  <br> Kps & Time                       |SHITOMASI + ORB <br> Kps & Time       | SHITOMASI + FREAK    <br> Kps & Time                  | SHITOMASI + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- |----------------------- | ----------------------- | ----------------------- |
| 1      | 65 & 18.931 | 83 & 18.817|75  & 22.136 |65 & 18.841 | 83	& 12.198 |
| 2      | 63 & 19.488 | 78 & 18.663|77 & 19.130 | 66 & 18.819 | 81 &	16.487 |
| 3      | 67 & 17.674 |  85 & 17.119|81 & 19.017 | 63 & 13.570 | 85 &	12.379 |
| 4      | 65 & 17.677  |  80 & 16.800|80 & 18.836| 66 & 12.911 | 81 & 12.373 |
| 5      | 70 & 17.194  |  86 & 17.824|85 & 19.678 | 66 & 14.837 | 86 &	13.141 |
| 6      | 64 & 19.185  | 85 & 17.852|77 & 17.912 | 59 & 12.720 | 84 & 12.142 |
| 7      | 64 & 17.192  | 80 & 17.295|76 & 17.695 | 63 & 12.357 | 77 &	13.319 |
| 8      | 69 & 17.145  | 87 & 16.471|84 & 17.299 | 71 & 12.311 | 87 &	15.856 |
| 9      | 70 & 19.973  | 87 & 17.092|83 & 18.169 | 66 & 12.311 | 88 &	13.707|
---

|  Images      | HARRIS + BRISK <br> Kps & Time               | HARRIS + BRIEF  <br> Kps & Time                       |HARRIS + ORB <br> Kps & Time       | HARRIS + FREAK    <br> Kps & Time                  | HARRIS + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- |----------------------- | ----------------------- | ----------------------- |
| 1      | 12 & 19.439 | 14 & 19.341|12  & 20.640 |13 & 21.635 | 14	& 20.354 |
| 2      | 10 & 22.559 | 11 & 19.341|12 &  19.630| 12 & 19.399 | 11 &	21.151 |
| 3      | 14 & 19.438 |  16 & 19.597|15 &  19.067| 15 & 14.130 | 16 &	22.083 |
| 4      | 16 & 19.117 |  21 & 19.714|19 &  18.740| 16 & 14.147 | 20 & 20.104 |
| 5      | 16 & 44.219  |  23 & 43.697|23 & 37.582 | 16 & 36.261 | 21 &	45.013 |
| 6      | 17 & 17.936  | 28 & 18.054|21 & 12.546 | 21 & 13.558| 23 & 20.573 |
| 7      | 15 & 22.980  | 16 & 22.540|15 & 17.326 | 12 & 17.395 | 13 &	22.307 |
| 8      | 22 & 20.025  | 25 & 16.471|25 & 19.000 | 21 & 17.037 | 24 &	21.071 |
| 9      | 21 & 27.815  | 24 & 25.935|23 & 24.701 | 19 & 23.052 | 23 &	25.404|
---

|  Images      | FAST + BRISK <br> Kps & Time               | FAST + BRIEF  <br> Kps & Time                       |FAST + ORB <br> Kps & Time       | FAST + FREAK    <br> Kps & Time                  | FAST + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- |----------------------- | ----------------------- | ----------------------- |
| 1      | 256 & 7.642 | 320 & 7.556|307  & 7.448 |251 & 7.447 | 316	& 8.567 |
| 2      | 243 & 7.227 | 332 & 7.284|308 &  7.089| 247 & 7.171 | 325 &	8.792 |
| 3      | 241 & 7.156 |  299 & 7.228|298 &  6.931| 233 & 7.261 | 297 &	8.550 |
| 4      | 239 & 7.274 |  331 & 6.697|321 &  7.064| 255 & 6.751 | 311 &9.356 |
| 5      | 215 & 6.792  |  276 & 6.683|283 & 6.559 | 231 & 6.668 | 291 &	7.564 |
| 6      | 251 & 7.189 | 327 & 7.194|315 & 7.310 | 265 & 7.529| 326 & 8.687|
| 7      | 248 & 6.843 | 324 & 6.889|323 & 6.719 | 251 & 7.236 | 315 &	8.505|

---

|  Images      | BRISK + BRISK <br> Kps & Time               | BRISK + BRIEF  <br> Kps & Time                       |BRISK + ORB <br> Kps & Time       | BRISK + FREAK    <br> Kps & Time                  | BRISK + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- |----------------------- | ----------------------- | ----------------------- |
| 1      | 171 & 451.813 | 178 & 453.465|162  & 435.504 |160 & 449.36 | 182	& 447.584 |
| 2      | 176 & 452.252| 205 & 457.100|175 & 450.987| 177 & 483.626 | 193 &	473.527 |
| 3      | 157 & 440.112 |  185 & 450.748|158 & 448.6541| 155 & 457.869| 169 &	444.082 |
| 4      | 176 & 437.355 |  179 & 444.607|167 &  453.459| 173 & 445.119 | 183 & 442.719 |
| 5      | 174 & 438.892|  183 & 448.64|160 & 444.772| 161 & 450.824 | 171 &	439.016 |
| 6      | 188 & 440.715 | 195 & 445.708|182 & 447.784 | 183 & 442.273| 195 & 452.562|
| 7      | 173 & 436.512 | 207 & 442.067|167 & 445.621| 169 &  441.397| 194 &	441.616|
| 8      | 171 & 438.953| 189 & 443.321|171 & 451.685 | 178 & 437.142| 176 &	453.06 |
| 9      | 184 & 441.888  | 183 & 453.143|172 & 439.407 | 168 & 443.227| 183 &	442.494|
---

|  Images      | ORB + BRISK <br> Kps & Time               | ORB + BRIEF  <br> Kps & Time                       |ORB + ORB <br> Kps & Time       | ORB + FREAK    <br> Kps & Time                  | ORB + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- |----------------------- | ----------------------- | ----------------------- |
| 1      | 73 & 10.507 | 49 & 9.324|67  & 9.127 |42 & 9.448 | 67	& 8.066 |
| 2      | 74 & 9.865| 43 & 9.508|70 & 9.778| 36 & 8.910| 79 &	10.145 |
| 3      | 79 & 8.645 |  45 & 8.918|72 & 8.912| 44 & 8.500| 78 &	9.351 |
| 4      | 85 & 10.984 |  59 & 8.393|84 & 8.557| 47 & 9.181| 79 & 8.370 |
| 5      | 79 & 8.704|  53 & 8.6267|91 & 8.796| 44 & 8.251 | 82 &	8.318 |
| 6      | 92 & 8.722 | 78 & 8.343|101 & 10.414| 51 & 7.934| 95 & 8.526|
| 7      | 90 & 8.215 | 68 & 8.6807|92 & 9.077| 52 &  8.127| 95 &	8.551|
| 8      | 88 & 8.337| 84 & 8.352|93 & 8.898 | 48 & 9.752| 94 &	8.507 |
| 9      | 91 & 8.364  | 66 & 9.743|93 & 9.145 | 56 & 8.478| 94 &	8.490|
---

|  Images      | AKAZE + BRISK <br> Kps & Time               | AKAZE + BRIEF  <br> Kps & Time                       |AKAZE + ORB <br> Kps & Time       | AKAZE + FREAK    <br> Kps & Time                  |  AKAZE + AKAZE    <br> Kps & Time                  |AKAZE + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- |-----------------------|----------------------- | ----------------------- | ----------------------- |
| 1      | 137 &  129.036 |141 & 125.476|131  & 123.528 |126 & 119.062 | 138	& 104.339|134	& 118.719 |
| 2      | 125 & 122.584| 134 & 127.807|129 & 139.982| 129 & 118.754| 138 &	120.456 |134	& 119.154 |
| 3      | 129 & 120.762|  131 & 133.566 | 127 & 137.183| 127 & 121.324| 133 &	115.556 |130	& 114.267 |
| 4      | 129 & 120.601 |  130 & 131.337|117 & 126.115| 121 & 125.028| 127 & 110.992 |136	& 120.959 |
| 5      | 131 & 116.903|  134 & 128.417|130 & 123.657 |122 & 117.334 | 129 &	111.057 |137	&  121.549 |
| 6      | 132 & 114.442| 146 & 129.671|131 & 120.511| 133 & 121.184| 146 & 115.242|147	& 124.242 |
| 7      | 142 & 113.449| 150 & 120.449|137 & 122.709| 144 &  114.791| 147 &	114.672|147	& 110.892 |
| 8      | 146 & 114.111 |148 & 123.204|135 & 124.721| 147 & 110.587| 151 &	117.976 |154	& 125.486 |
| 9      | 144 & 115.640 | 152 & 126.266|145 & 117.292 |138 & 116.13| 150 &	118.026|151	& 112.126 |
---

|  Images      | SIFT + BRISK <br> Kps & Time               | SIFT + BRIEF  <br> Kps & Time                       | SIFT + FREAK    <br> Kps & Time                  | SIFT + SIFT <br> Kps & Time                        |
| ------------- | ----------------------- | ----------------------- | ----------------------- | ----------------------- |
| 1      | 64 & 171.63 | 86 & 172.708|65  & 163.179 |82 & 130.101 | 
| 2      | 66 & 162.141| 78 & 174.647|72 & 171.405| 81 & 150.547| 
| 3      | 62 & 138.45 |  76 & 164.687|64 & 165.124| 85 & 130.839|
| 4      | 66 & 136.504 |  85 & 160.176|66 & 166.534| 93 & 136.339| 
| 5      | 59 & 132.045|  69 & 164.589|59 & 170.896| 90 & 133.846 | 
| 6      | 64 & 134.541 | 74 &  161.622|59 & 161.973| 81 & 135.649| 
| 7      | 67 & 133.667 | 76 & 159.225|64 & 179.077| 82 &  132.489| 
| 8      | 67 & 130.65 | 70 & 167.534|65 & 179.077 | 102 & 141.395| 
| 9      | 80 & 131.758  | 88 & 170.153|79 & 179.077 | 104 & 132.133|
---
####  MP.9 Performance Evaluation 3
From the above tabular results, it can be noticed that the FAST detectors are the fastest in terms of speed.  
The TOP 3 detector / descriptor combinations for the purpose of detecting keypoints on preceding vehicle is given below. 

|         | Detector / Descriptor           | 
| ------------- |:-------------:|
| TOP 1      | FAST+ BRIEF |
| TOP 2     | FAST + BRISK      | 
| TOP 3 | FAST + ORB      |  
---

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
