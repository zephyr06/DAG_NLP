## Must do things before any test:
### General settings:
* Update `PROJECT_PATH` in `sources/Utils/Parameters.cpp` to correct path


### runVerucchiOnlyRTAS2023 settings
* Update `ROOT_PATH` and `TaskNumberArray` in `CompareWithBaseline/VerucchiOnlyRTAS2023/runVerucchiOnlyRTAS2023.sh` for verucchi only tests
* Make sure you are using `RTDATasksets` in `TASKSETS_PATH` of `runVerucchiOnlyRTAS2023.sh`


### WangNLP settings
* Make sure GTASM NLP timeout is 3
* Use 1e-3 as relativeErrorTolerance in `sources/Baseline/Wang21/sources/parameters.yaml`
* Update `ROOT_PATH` and `TaskNumberArray` in `CompareWithBaseline/WangNLPOnlyRTAS2023/runWangNLPOnlyRTAS2023.sh` for WangNLP only tests
* Make sure you are using `SensorFusionTasksets` in `TASKSETS_PATH` of `runWangNLPOnlyRTAS2023.sh`


### runOrderOptOnlyRtdaRTAS2023 settings
* Update `ROOT_PATH` and `TaskNumberArray` in `CompareWithBaseline/OrderOptOnlyRtdaRTAS2023/runOrderOptOnlyRtdaRTAS2023.sh` for OrderOpt only tests
* Make sure you are using `RTDATasksets` in `TASKSETS_PATH` of `runOrderOptOnlyRtdaRTAS2023.sh`


### runOrderOptOnlySensorFusionRTAS2023 settings
* Update `ROOT_PATH` and `TaskNumberArray` in `CompareWithBaseline/OrderOptOnlySensorFusionRTAS2023/runOrderOptOnlySensorFusionRTAS2023.sh` for OrderOpt only tests
* Make sure you are using `SensorFusionTasksets` in `TASKSETS_PATH` of `runOrderOptOnlySensorFusionRTAS2023.sh`


### batch settings
* Update `ROOT_PATH` in `CompareWithBaseline/RTDA2CoresPerformance/CompareRTDA2CoresPerformance.sh` for RTDA test
* Update `ROOT_PATH` in `CompareWithBaseline/SensorFusion2CoresPerformance/CompareSensorFusion2CoresPerformance.sh` for Sensor Fusion test
* Update `ROOT_PATH` in `CompareWithBaseline/build_tar_ball_for_backup.sh` to back up tar balls