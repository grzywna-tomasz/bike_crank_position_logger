include Makefile_cfg

# default action: build all
all: $(BUILD_DIR)/$(TARGET_NAME).elf $(BUILD_DIR)/$(TARGET_NAME).hex

# Display or hide compilation command
ifeq ($(VERBOSE), 1)
    VB =
else
    VB = @
endif

OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCE:../%.c=%.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCE:../%.s=%.o))

$(BUILD_DIR)/%.o: ../%.c Makefile Makefile_cfg Makefile_scope
	$(VB)echo "compiling $(notdir $<)"
	$(VB)mkdir -p $(dir $@)
	$(VB)$(CC) -c $(C_FLAGS) $< -o $@

$(BUILD_DIR)/%.o: ../%.s Makefile Makefile_cfg Makefile_scope
	$(VB)echo "compiling $(notdir $<)"
	$(VB)$(AS) -c $(ASM_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET_NAME).elf: $(OBJECTS) Makefile Makefile_cfg Makefile_scope
	$(VB)echo "linking $(notdir $@)"
	$(VB)$(CC) $(OBJECTS) $(LD_FLAGS) -o $@
	$(VB)$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf Makefile Makefile_cfg Makefile_scope
	$(VB)echo "creating $(notdir $@)"
	$(VB)$(CP) -O ihex $< $@

$(BUILD_DIR):
	$(VB)mkdir $@	

clean:
	rm -fR $(BUILD_DIR)

-include $(OBJECTS:%.o=%.d)