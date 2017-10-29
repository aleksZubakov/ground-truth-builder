all: hedgehog_example

hedgehog_example: main.o hedgehog_proxy.o
	gcc main.o hedgehog_proxy.o -o hedgehog_example

main.o: main.cpp
	gcc -c main.cpp

hedgehog_proxy.o: hedgehog_proxy.cpp
	gcc -c hedgehog_proxy.cpp

clean:
	rm -f *o
