#include "common_test_header.h"
#include <interpreter.hpp>
#include "test_struct.h"

using namespace std;

TEST_CASE("pod struct parse")
{
	REQUIRE(args.argc() > 1);

	ezcfg::Interpreter itp(args.argv()[1]);
	REQUIRE(itp);

	TestStr rr;
	itp.parse(rr);

	CHECK(rr.a == 1);
	CHECK(rr.b == 6.4f);
	CHECK(rr.c == 115);
	CHECK(rr.d[0] == 'd');
	CHECK(rr.d[1] == 'e');
	CHECK(rr.e[0] == string("aa"));
	CHECK(rr.e[1] == string("ss"));
	CHECK(rr.f == "hello world");
	CHECK(rr.g.size() == 3);
	CHECK(rr.g[0] == 1);
	CHECK(rr.g[1] == 2);
	CHECK(rr.g[2] == 3);
	CHECK(rr.h.size() == 2);
	CHECK(rr.h[8.6] == 789);
	CHECK(rr.h[9.654] == 568);
}
