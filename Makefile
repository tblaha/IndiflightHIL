RM=rm -rf
CMAKE=cmake
MAKE=make -j12
MKDIR=mkdir -p
CP=cp -r
CD=cd
PYTHON=python3

.DEFAULT_GOAL := gz-plugins

PLUGIN_PATH := gz-plugins
PLUGIN_BIN_DIR := $(PLUGIN_PATH)/libexec
PLUGIN_DIRS := $(filter-out $(PLUGIN_BIN_DIR)/,$(wildcard $(PLUGIN_PATH)/*/))

$(addsuffix /build/*.so, $(PLUGIN_DIRS)) : $(addsuffix /CMakeLists.txt, $(PLUGIN_DIRS)) $(addsuffix /*.cc, $(PLUGIN_DIRS))
	$(MKDIR) $(PLUGIN_BIN_DIR)
	$(MKDIR) $(dir $@)
	$(CD) $(dir $@) && $(CMAKE) ../ && $(MAKE)
	$(CP) $(dir $@)/*.so $(PLUGIN_BIN_DIR)/

gz-plugins : $(addsuffix /build/*.so, $(PLUGIN_DIRS))

all : gz-plugins

# -------------------- #
clean-gz-plugins : 
	$(foreach DIR, $(PLUGIN_DIRS), $(RM) $(DIR)/build;)
	$(RM) $(PLUGIN_BIN_DIR)

deep-clean : clean-gz-plugins
