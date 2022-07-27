CIRCUITPY_CREATOR_ID =  0x0000239A
CIRCUITPY_CREATION_ID = 0x00320001

IDF_TARGET = esp32

INTERNAL_FLASH_FILESYSTEM = 1
LONGINT_IMPL = MPZ

# The default queue depth of 16 overflows on release builds,
# so increase it to 32.
CFLAGS += -DCFG_TUD_TASK_QUEUE_SZ=32

CIRCUITPY_ESP_FLASH_MODE = dio
CIRCUITPY_ESP_FLASH_FREQ = 80m
CIRCUITPY_ESP_FLASH_SIZE = 16MB

CIRCUITPY_SHARPDISPLAY = 0
CIRCUITPY_STATUS_BAR = 0
CIRCUITPY_WEB_WORKFLOW = 0
CIRCUITPY_STAGE = 1

# Include these Python libraries in firmware.
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_NeoPixel
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_Display_Text
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_FakeRequests
FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_Requests
