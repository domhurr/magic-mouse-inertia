CC ?= cc
PKG_CONFIG ?= pkg-config

TARGET := magic-mouse-inertia
SOURCE := magic-mouse-inertia.c

CPPFLAGS += $(shell $(PKG_CONFIG) --cflags libevdev)
CFLAGS ?= -O2
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror
LDLIBS += $(shell $(PKG_CONFIG) --libs libevdev) -lm

.PHONY: all check clean

all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

check: $(TARGET)
	./$(TARGET) --help >/dev/null

clean:
	rm -f $(TARGET)
