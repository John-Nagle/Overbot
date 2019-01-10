all:	printlocalnode chrootchroot chrootlocal chrootacross chrootread

chroootread:	chrootread.c
	gcc -o chrootread chrootread.c

chrootacross:		chrootacross.c
	gcc -o chrootacross chrootacross.c

chrootlocal: 		chrootlocal.c
	gcc -o chrootlocal chrootlocal.c

printlocalnode:		printlocalnode.c
	gcc -o printlocalnode printlocalnode.c

chrootchroot:	chrootchroot.c
	gcc -o chrootchroot chrootchroot.c

clean:
	\rm *.o printlocalnode chrootlocal chrootacross chrootchroot chrootread

