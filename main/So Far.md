1. To add cpp library and use them to ESP IDF
	a. add "extern "C" {void app_main(void);}"
	b. change the main.c to main.cpp
	c. all cpp methods/task now need to be void and declared as "sometask(void*)"
	d. no task/method initialization, all methods should have {}

2. Eclipse IDE no auto save (TODO), remember to save before building