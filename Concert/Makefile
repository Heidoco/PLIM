CFLAGS= -Wall -m64 -g -w
#CFLAGS= -Wall -m64 -o2 
CXX=g++

ILOG= /opt/ibm/ILOG/CPLEX_Studio2211
CPPFLAGS= -DIL_STD -I$(ILOG)/cplex/include -I$(ILOG)/concert/include
CPLEXLIB=-L$(ILOG)/cplex/lib/x86-64_linux/static_pic -lilocplex -lcplex -L$(ILOG)/concert/lib/x86-64_linux/static_pic -lconcert -lm -lpthread -ldl

 
comp-c: 
	$(CXX) $(CFLAGS) $(CPPFLAGS) -o ncondireta  conexaodireta.cpp  $(CPLEXLIB) 



run:comp-mono
	./mono FL-50-50	
clean:
	rm -f  *.out *.aux *.log *.nav *.snm *.out *.toc 
