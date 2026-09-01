# NUBots - New Segmentation Model

## Comparative Analysis

### Download data/ && models/

The following is the arg convention for downloading new models

```
./scripts/add_models.sh <URL> <DESTINATION>
```

The following will ensure that the data downloader helper &  script has the correct permissions before running. It is recommended to run the following before continuing.

```
chmod 777 scripts/ensure_dataset.sh
chmod 777 scripts/add_models.sh
./scripts/ensure_dataset.sh
./scripts/add_models.sh https://github.com/ultralytics/ultralytics ultralytics
./scripts/add_models.sh https://github.com/roboflow/rf-detr rf-detr
./scripts/add_models.sh https://github.com/PaddlePaddle/PaddleSeg pp-lite-seg
./scripts/add_models.sh https://github.com/CoinCheung/BiSeNet bisenetv2
```

