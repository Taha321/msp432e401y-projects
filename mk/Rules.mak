
.PHONY: all clean lib veryclean

TARGET_DIR?=./

all: ${PROG}.elf

clean:
	rm -rf ${OBJS}
	rm -rf ${TARGET_DIR}/${PROG}.*

${PROG}.elf : ${OBJS}
	${CC} ${LDFLAGS}  $^ -o ${TARGET_DIR}/$@ 

%.o : %.c
	${CC} ${CFLAGS} -c $< -o $@ ${patsubst %, -I%, ${INCLUDE_DIR}} 

%.o : %.cpp
	${CPP} ${CPPFLAGS} -c $< -o $@ ${patsubst %, -I%, ${INCLUDE_DIR}} 

