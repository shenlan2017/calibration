#define DOCTEST_CONFIG_NO_UNPREFIXED_OPTIONS
#define DOCTEST_CONFIG_IMPLEMENT

#include "common_test_header.h"

dt_removed args;

int main(int argc, char** argv)
{
	doctest::Context context;
	context.applyCommandLine(argc, argv);
	args = argv;
	return context.run();
}
