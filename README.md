# Orientation prediction
This repository includes the implementation for [Orientation Prediction for VR and AR Devices Using Inertial Sensors Based on Kalman-like Error Compensation](https://ieeexplore.ieee.org/document/9931011)
(IEEE Access 2022).

# Abstract
We propose an orientation prediction algorithm based on Kalman-like error compensation for virtual reality (VR) and augmented reality (AR) devices using measurements of an inertial measurement unit (IMU), which includes a tri-axial gyroscope and a tri-axial accelerometer.

## Block diagram
![block_diagram](./figures/algorithm.png)

### Experimental results

**Comparison of RMSE values for the Euler angles**
![results](./figures/results.PNG)


**Comparison of the predicted Euler angles on the LOPSI test dataset**
![results](./figures/LOPSI.PNG)


**Comparison of the predicted Euler angles on the FKF test dataset**
![results](./figures/FKF.PNG)


**Comparison of the predicted Euler angles on the spiralStairs test dataset**
![results](./figures/spiralStairs.PNG)
## Citation

If you find this repo helpful, please consider citing:

```
@ARTICLE{9931011,
  author={Hue Dao, Le Thi and Mai, Truong Thanh Nhat and Hong, Wook and Park, Sanghyun and Kim, Hokwon and Lee, Joon Goo and Kim, Min-Seok and Lee, Chul},
  journal={IEEE Access}, 
  title={Orientation Prediction for VR and AR Devices Using Inertial Sensors Based on Kalman-Like Error Compensation}, 
  year={2022},
  volume={10},
  number={},
  pages={114306-114317},
  doi={10.1109/ACCESS.2022.3217555}}
```
