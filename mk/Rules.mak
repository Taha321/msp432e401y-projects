
.PHONY: all clean lib veryclean

TARGET_DIR?=./

all: ${PROG}.elf

clean:
	rm -rf ${OBJS}
	rm -rf ${TARGET_DIR}/${PROG}.*
	rm -rf *.map

${PROG}.elf : ${OBJS}
	${CC} ${LDFLAGS}  $^ -o ${TARGET_DIR}/$@ 

%.o : %.c
	@echo "Compiling C file: $<"
	${CC} ${CFLAGS} -c $< -o $@ ${patsubst %, -I%, ${INCLUDE_DIR}} 

%.o : %.cpp
	@echo "Compiling C file: $<"
	${CPP} ${CPPFLAGS} -c $< -o $@ ${patsubst %, -I%, ${INCLUDE_DIR}} 

