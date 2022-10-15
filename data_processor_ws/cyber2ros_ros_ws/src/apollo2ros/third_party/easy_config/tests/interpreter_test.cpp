#include "common_test_header.h"
#include <interpreter.hpp>
#include "test_struct.h"
using namespace std;

namespace ezcfg
{
    template<>
    void Interpreter::parserDispatcher<::TestStr>(TestStr &data)
    {
        lex.match(Token::L_BRACE);
        lex.match(Token::DOT);
        lex.matchID("a");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.a);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("b");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.b);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("c");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.c);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("d");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.d);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("e");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.e);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("f");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.f);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("g");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.g);
        lex.match(Token::COMMA);
        lex.match(Token::DOT);
        lex.matchID("h");
        if (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.option(Token::EQU);
        parserDispatcher(data.h);
        lex.option(Token::COMMA);
        lex.match(Token::R_BRACE);
    }
}

TEST_CASE("test struct parse")
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

TEST_CASE("expression parse test")
{
	ezcfg::Interpreter itp("(1), (1.5 + 1) * 2; 33 * -33",false);
	double a = itp.parseExpression();
	CHECK(a == ((1), (1.5 + 1) * 2));
	int b = itp.parseExpression();
	CHECK(b == (33 * -33));
}
