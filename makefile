# ==========================================
# Configuration & Flags
# ==========================================
# Silence "make[1]: Entering directory" messages
MAKEFLAGS    += --no-print-directory

PYTHON       := python
DOWNLOADER   := downloader.bat
CP_CONTENTS  := cp-contents.bat

# User Configuration (Lowercase as requested)
folder       ?= test
esp_ip       ?= 

# File paths
BIN_FILE     := response.bin
CSV_FILE     := sensor_data.csv
DATA_OUTPUT  := data/$(folder)/measured.txt

# Phony Targets (Commands)
.PHONY: all clean download roll pitch test a-roll a-pitch a-test \
        force-start force-process help filter

# ==========================================
# 1. Main Analysis Tasks (Step 4)
# ==========================================

all: $(DATA_OUTPUT) kf_filter.py
	@echo.
	@echo "   [ STEP 4 ] Running Kalman Filter with dataset: $(folder)"
	@echo "   --------------------------------------------"
	@$(PYTHON) kf_filter.py $(folder)
	@echo.

# ==========================================
# 2. File Processing Chain (Steps 2 & 3)
# ==========================================

# Step C: Copy to specific folder (Step 3)
# Depends on CSV_FILE.
$(DATA_OUTPUT): $(CSV_FILE)
	@echo.
	@echo "   [ STEP 3 ] Archiving Data"
	@echo "   Target: data/$(folder)/measured.txt"
	@echo "   --------------------------------------------"
	@$(CP_CONTENTS) $(folder)

# Step B: Convert Bin to CSV (Step 2)
# Depends on BIN_FILE.
# We depend on 'force-process' so this ALWAYS runs, ensuring you see the message.
$(CSV_FILE): $(BIN_FILE) bin2csv.py force-process
	@echo.
	@echo "   [ STEP 2 ] Converting Binary to CSV"
	@echo "   Input: $(BIN_FILE)"
	@echo "   --------------------------------------------"
	@$(PYTHON) bin2csv.py

# Step A: The Binary Source (Step 1)
# Depends on 'force-start' so this ALWAYS runs to print the status message.
$(BIN_FILE): force-start
ifdef esp_ip
	@echo.
	@echo "   [ STEP 1 ] Connection Detected"
	@echo "   Downloading from ESP32 at $(esp_ip)..."
	@echo "   --------------------------------------------"
	@$(DOWNLOADER) $(esp_ip)
else
	@echo.
	@echo "   [ STEP 1 ] Local Mode"
	@echo "   Using cached file: $(BIN_FILE)"
	@echo "   --------------------------------------------"
	@if not exist $(BIN_FILE) echo [ ERROR ] $(BIN_FILE) not found! && exit 1
endif

# ==========================================
# 3. Helpers & Animation
# ==========================================

# Dummy targets to force recipe execution
force-start: ;
force-process: ;

# Animation Targets
a-folder: 
	@echo "   [ ANIMATION ] Launching 3D Box for folder: $(folder)..."
	@$(PYTHON) box_animation.py $(folder)

a-roll:  
	@echo "   [ ANIMATION ] Launching 3D Box for Roll..."
	@$(PYTHON) box_animation.py roll

a-pitch: 
	@echo "   [ ANIMATION ] Launching 3D Box for Pitch..."
	@$(PYTHON) box_animation.py pitch

a-test: 
	@echo "   [ ANIMATION ] Launching 3D Box for Test..."
	@$(PYTHON) box_animation.py test

clean:
	@echo.
	@echo "   [ CLEAN ] Removing temporary files..."
	@rm -f *.pyc
	@rm -rf __pycache__
	@echo "   Done."


filter: kf_filter.py
	@echo.
	@echo "   [ FILTER ] Running Kalman Filter with dataset: $(folder)"
	@echo "   --------------------------------------------"
	@$(PYTHON) kf_filter.py $(folder)
	@echo.

# ==========================================
# 4. Detailed Help Manual
# ==========================================
help:
	@echo.
	@echo "   ======================================================================="
	@echo "                       KALMAN FILTER PROJECT MANUAL"
	@echo "   ======================================================================="
	@echo "   This Makefile automates the sensor data pipeline:"
	@echo "   Download -> Convert (Bin to CSV) -> Archive (Copy to Folder) -> Filter"
	@echo.
	@echo "   USAGE SYNTAX:"
	@echo "      make [target] [variable=value]"
	@echo "   -----------------------------------------------------------------------"
	@echo "   CONFIGURATION VARIABLES"
	@echo "   -----------------------------------------------------------------------"
	@echo "      esp_ip          : The IP address of the ESP32."
	@echo "                        - If SET: Forces a download of response.bin."
	@echo "                        - If EMPTY: Uses the existing local response.bin."
	@echo.
	@echo "      folder          : Destination subfolder inside 'data/'."
	@echo "                        - Default is 'test'."
	@echo "                        - Example: make test folder=pitch_data"
	@echo "   ======================================================================="

