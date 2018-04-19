TARGET :=main 
CC := g++
#注意每行后面不要有空格，否则会算到目录名里面，导致问题  
SRC_DIR = src
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
DEPS_DIR  = $(BUILD_DIR)/deps
#这里添加其他头文件路径
INC_DIR =-I./include
#这里添加编译参数
CC_FLAGS := $(INC_DIR) -g -std=c++11
LNK_FLAGS := -L/usr/local/lib -L./lib -lpthread -ljson -lboost_system -lboost_thread -lboost_chrono
#这里递归遍历3级子目录  
DIRS := $(shell find $(SRC_DIR) -maxdepth 3 -type d)
#将每个子目录添加到搜索路径  
VPATH = $(DIRS)
#查找src_dir下面包含子目录的所有cpp文件  
SOURCES   = $(foreach dir, $(DIRS), $(wildcard $(dir)/*.cpp))
OBJS   = $(addprefix $(OBJ_DIR)/,$(patsubst %.cpp,%.o,$(notdir $(SOURCES))))
DEPS  = $(addprefix $(DEPS_DIR)/, $(patsubst %.cpp,%.d,$(notdir $(SOURCES))))
$(TARGET):$(OBJS)
	$(CC) $^ $(LNK_FLAGS) -o $@
#编译之前要创建OBJ目录，确保目录存在
$(OBJ_DIR)/%.o:%.cpp
	if [ ! -d $(OBJ_DIR) ]; then mkdir -p $(OBJ_DIR); fi;\
	$(CC) -c $(CC_FLAGS) -o $@ $<
#编译之前要创建DEPS目录，确保目录存在
#前面加@表示隐藏命令的执行打印  
$(DEPS_DIR)/%.d:%.cpp
	@if [ ! -d $(DEPS_DIR) ]; then mkdir -p $(DEPS_DIR); fi;\
	set -e; rm -f $@;\
	$(CC) -MM $(CC_FLAGS) $< > $@.;\
#	sed's,$*\.o[:]*,$(OBJ_DIR)/\1.o $@:,g'<$@.> $@;\
	rm -f $@.
#前面加-表示忽略错误
-include $(DEPS)
.PHONY : clean
clean:
	rm -rf $(BUILD_DIR) $(TARGET)  
