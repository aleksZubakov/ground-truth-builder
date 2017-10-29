all: hedgehog_example

hedgehog_example:
	g++ -g -pthread -c marvelmind.c -o marvelmind.o
	g++ -c hedgehog_proxy.c -o hedgehog_proxy.o
	g++ -g -pthread -c main.c -o main.o
	g++ -o hedgehog_example main.o hedgehog_proxy.o marvelmind.o -pthread

clean:
	rm -rf *.o
