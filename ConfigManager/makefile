
OBJECTS = ConfigParameter.o ConfigTree.o ConfigStorageManager.o ConfigManager.o

ConfigSystemTest: $(OBJECTS) TestConfig.o
	g++ $^ -o $@

Testing_MM: $(OBJECTS) Testing_MM.o
	g++ $^ -lrt -o $@

clean: 
	rm *.o ConfigSystemTest Testing_MM
