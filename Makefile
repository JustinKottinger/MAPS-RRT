
EXE := Planner

SRC_DIR := src
OBJ_DIR := obj

SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

CPPFLAGS := -std=c++11
CFLAGS   := -I/opt/local/var/macports/distfiles/ompl/omplapp-1.4.2-Source/ompl/src/
LDFLAGS  := -L/opt/local/lib -lompl -lboost_serialization
# LDLIBS   := -lm
CC = clang++

.PHONY: all clean

all: $(EXE)


$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@


$(OBJ_DIR):
	@mkdir $@

clean:
	$(RM) $(OBJ)