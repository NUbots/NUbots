# NUBots - New Segmentation Model

## Download data/ && models/

The following is the arg convention for downloading new models

```
./shared/ensure_models.sh <URL> <DESTINATION>
```

The following will ensure that the data downloader helper &  script has the correct permissions before running. It is recommended to run the following before continuing.

```
chmod 777 shared/ensure_dataset.sh
chmod 777 shared/ensure_models.sh
./shared/ensure_dataset_.sh
./shared/ensure_models.sh https://github.com/ultralytics/ultralytics
./shared/ensure_models.sh https://github.com/roboflow/rf-detr rf-detr
./shared/ensure_models.sh https://github.com/PaddlePaddle/PaddleSeg pp-lite-seg
./shared/ensure_models.sh https://github.com/CoinCheung/BiSeNet BiSeNetv2
```

