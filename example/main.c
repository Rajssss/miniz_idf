

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "miniz.h"
#include "sdkconfig.h"

#ifndef CONFIG_ESP32_SPIRAM_SUPPORT
#pragma message("This utility will not work without PSRAM")
#endif

static const char *TAG = "miniz_test";

const int iXmax = 800;
const int iYmax = 800;
const uint32_t rawImg = iXmax * iYmax *3;

typedef struct
{
  uint8_t r, g, b;
} rgb_t;


static void hsv_to_rgb(int hue, int min, int max, rgb_t *p)
{
  const int invert = 0;
  const int saturation = 1;
  const int color_rotate = 0;

  if (min == max) max = min + 1;
  if (invert) hue = max - (hue - min);
  if (!saturation) {
    p->r = p->g = p->b = 255 * (max - hue) / (max - min);
    return;
  }
  double h = fmod(color_rotate + 1e-4 + 4.0 * (hue - min) / (max - min), 6);
  double c = 255.0f * saturation;
  double X = c * (1 - fabs(fmod(h, 2) - 1));

  p->r = p->g = p->b = 0;

  switch((int)h) {
  case 0: p->r = c; p->g = X; return;
  case 1:  p->r = X; p->g = c; return;
  case 2: p->g = c; p->b = X; return;
  case 3: p->g = X; p->b = c; return;
  case 4: p->r = X; p->b = c; return;
  default:p->r = c; p->b = X;
  }
}

unsigned long IRAM_ATTR millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000);
}


int writePng(const char* pFilename)
{
  // Image resolution

  uint32_t time1 = millis();
  int iX, iY;
  const double CxMin = -2.5;
  const double CxMax = 1.5;
  const double CyMin = -2.0;
  const double CyMax = 2.0;

  double PixelWidth = (CxMax - CxMin) / iXmax;
  double PixelHeight = (CyMax - CyMin) / iYmax;

  // Z=Zx+Zy*i  ;   Z0 = 0
  double Zx, Zy;
  double Zx2, Zy2; // Zx2=Zx*Zx;  Zy2=Zy*Zy

  int Iteration;
  const int IterationMax = 200;

  // bail-out value , radius of circle
  const double EscapeRadius = 2;
  double ER2=EscapeRadius * EscapeRadius;
  uint8_t *pImage = (uint8_t *)malloc(rawImg);

  if (!pImage) {
      ESP_LOGE(TAG,"Failed to allocate memory for image");
      return 0;
  }
  // world ( double) coordinate = parameter plane
  double Cx,Cy;

  int MinIter = 9999, MaxIter = 0;

  for(iY = 0; iY < iYmax; iY++)
  {
    Cy = CyMin + iY * PixelHeight;
    if (fabs(Cy) < PixelHeight/2)
      Cy = 0.0; // Main antenna

    for(iX = 0; iX < iXmax; iX++)
    {
      uint8_t *color = pImage + (iX * 3) + (iY * iXmax * 3);

      Cx = CxMin + iX * PixelWidth;

      // initial value of orbit = critical point Z= 0
      Zx = 0.0;
      Zy = 0.0;
      Zx2 = Zx * Zx;
      Zy2 = Zy * Zy;

      for (Iteration=0;Iteration<IterationMax && ((Zx2+Zy2)<ER2);Iteration++)
      {
        Zy = 2 * Zx * Zy + Cy;
        Zx =Zx2 - Zy2 + Cx;
        Zx2 = Zx * Zx;
        Zy2 = Zy * Zy;
      };

      color[0] = (uint8_t)Iteration;
      color[1] = (uint8_t)Iteration >> 8;
      color[2] = 0;

      if (Iteration < MinIter)
        MinIter = Iteration;
      if (Iteration > MaxIter)
        MaxIter = Iteration;
    }
  }

  for(iY = 0; iY < iYmax; iY++)
  {
    for(iX = 0; iX < iXmax; iX++)
    {
      uint8_t *color = (uint8_t *)(pImage + (iX * 3) + (iY * iXmax * 3));

      uint Iterations = color[0] | (color[1] << 8U);

      hsv_to_rgb(Iterations, MinIter, MaxIter, (rgb_t *)color);
    }
  }
  ESP_LOGI(TAG,"Generation time %lu ms\n", millis()-time1);
  time1 = millis();

  // Now write the PNG image.
    size_t png_data_size = 0;
  {
    void *pPNG_data = tdefl_write_image_to_png_file_in_memory_ex(pImage, iXmax, iYmax, 3, &png_data_size, 6, 0);
    if (!pPNG_data)
      ESP_LOGE(TAG,"tdefl_write_image_to_png_file_in_memory_ex() failed!");
    else
    {
      ESP_LOGI(TAG,"Compression time %lu ms\n", millis()-time1);
      time1 = millis();

      FILE* pFile = fopen(pFilename, "w");
      if(pFile == NULL)
      {
          ESP_LOGE(TAG, "Failed to open file for writing");
          return -1;
      }

      fwrite((const void*)pPNG_data, 1, png_data_size, pFile);
      fclose(pFile);
      ESP_LOGI(TAG,"Wrote %s. Write time %lu ms\n", pFilename, millis()-time1);
    }

    free(pPNG_data);
  }

  free(pImage);
  return png_data_size;
}


void create_png_task(void *args)
{
	ESP_LOGI(TAG, "create_png_task run.");
    uint32_t file_size = writePng("/spiffs/mandelbrot.png");
    ESP_LOGI(TAG,"Compressed %u byte image to %u bytes\n", rawImg, file_size);

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


void app_main(void)
{
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "storage",
      .max_files = 5,
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    ret = esp_spiffs_format(conf.partition_label);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Format failed");
    } else {
        ESP_LOGI(TAG, "Format done");
    }

    ESP_LOGI(TAG,"Will use ~%u of %u bytes memory\n", rawImg * 2, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if((rawImg * 2) > heap_caps_get_free_size(MALLOC_CAP_SPIRAM))
    {
    	ESP_LOGE(TAG,"Not enough memory to build the image");
    }

    xTaskCreatePinnedToCore(create_png_task, "create_png_task", 1024*10, NULL, 2, NULL, 1);

    while(true)
    {
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

