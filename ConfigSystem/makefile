
OBJECTS = ConfigParameter.o      ConfigTree.o    \
          ConfigStorageManager.o ConfigManager.o \
          Module.o Configurable.o


TESTOBJECTS =          \
ConfigManager.o        \
ConfigParameter.o      \
ConfigStorageManager.o \
ConfigTree.o           \
Configurable.o         \
Module.o               \
Testing_MM.o           \
Testing_MM_globals.o   \
Testing_MM_Utils.o


ConfigSystemTest: $(OBJECTS) TestConfig.o
	g++ $^ -o $@

Testing_MM: $(TESTOBJECTS)
	g++ $^ -lrt -o $@
	# -lrt is for realtime extensions (for clock_gettime)

profileTests: $(OBJECTS) Testing_MM.o Testing_MM_Utils.o Testing_MM_globals.o
	# profile with Valgrind
	g++ $^ -lrt -o $@
	valgrind --tool=callgrind ./Testing_MM_prof
	rm -f callgrind.out.*
	callgrind_annotate callgrind.out.* > profile.txt
	
	# # Profile with gprof
	# g++ $(OBJECTS) Testing_MM.o -pg -lrt -o Testing_MM_prof
	# ./Testing_MM_prof
	# mv gmon.out gmon.sum
	# $(foreach var,$( 1  2  3  4  5  6  7  8  9 10    \
 #                    11 12 13 14 15 16 17 18 19 20    \
 #                    21 22 23 24 25 26 27 28 29 30    \
 #                    31 32 33 34 35 36 37 38 39 40    \
 #                    41 42 43 44 45 46 47 48 49 50),  \
 #        ./Testing_MM_prof                            \
 #        gprof -s Testing_MM_prof gmon.out gmon.sum   \
 #     )
	# gprof Testing_MM_prof gmon.sum > profile.txt


clean: 
	rm -f *.o ConfigSystemTest Testing_MM
