#include "common_test_header.h"
#include <interpreter.hpp>

TEST_CASE("lex")
{
	REQUIRE(args.argc() > 1);

	ezcfg::Lexer lex;
	lex.loadFile(args.argv()[1]);

	REQUIRE(lex);
	REQUIRE(lex.getToken() == ezcfg::Token::SCOPE);
	REQUIRE(lex.next() == ezcfg::Token::L_ANGLE_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::L_BRACE);
	REQUIRE(lex.next() == ezcfg::Token::L_BRACE);
	REQUIRE(lex.next() == ezcfg::Token::R_BRACE);
	REQUIRE(lex.next() == ezcfg::Token::R_BRACE);
	REQUIRE(lex.next() == ezcfg::Token::L_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::L_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::R_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::R_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::L_PARENTHESIS);
	REQUIRE(lex.next() == ezcfg::Token::L_PARENTHESIS);
	REQUIRE(lex.next() == ezcfg::Token::R_PARENTHESIS);
	REQUIRE(lex.next() == ezcfg::Token::R_PARENTHESIS);
	REQUIRE(lex.next() == ezcfg::Token::R_ANGLE_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::R_ANGLE_BRACKET);
	REQUIRE(lex.next() == ezcfg::Token::DOT);
	REQUIRE(lex.next() == ezcfg::Token::DOT);
	REQUIRE(lex.next() == ezcfg::Token::COMMA);
	REQUIRE(lex.next() == ezcfg::Token::COMMA);
	REQUIRE(lex.next() == ezcfg::Token::SEMICOLON);
	REQUIRE(lex.next() == ezcfg::Token::SEMICOLON);
	REQUIRE(lex.next() == ezcfg::Token::EQU);
	REQUIRE(lex.next() == ezcfg::Token::ADD);
	REQUIRE(lex.next() == ezcfg::Token::ADD);
	REQUIRE(lex.next() == ezcfg::Token::SUB);
	REQUIRE(lex.next() == ezcfg::Token::SUB);

	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == 'a');
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == 'n');
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == '*');
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == '\n');
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == '\075');
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == '\55');
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(char(lex.getNumber()) == '\x0f');
	REQUIRE(lex.next() == ezcfg::Token::STR);
	REQUIRE(lex.getTokenText() == "this is string\n\"esc\"");

	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(bool(lex.getNumber()) == true);
	REQUIRE(lex.getTokenText() == "true");
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(bool(lex.getNumber()) == false);
	REQUIRE(lex.getTokenText() == "false");

	REQUIRE(lex.next() == ezcfg::Token::STR);
	REQUIRE(lex.getTokenText() == "\n)\\\na\"\n");

	REQUIRE(lex.next() == ezcfg::Token::ID);
	REQUIRE(lex.getTokenText() == "_0identify");

	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(size_t(lex.getNumber()) == 0666);
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(size_t(lex.getNumber()) == 96354);
	REQUIRE(lex.next() == ezcfg::Token::INT);
	REQUIRE(size_t(lex.getNumber()) == 0xffe55);
	REQUIRE(lex.next() == ezcfg::Token::FLOAT);
	REQUIRE(double(lex.getNumber()) == 3.6);
	REQUIRE(lex.next() == ezcfg::Token::FLOAT);
	REQUIRE(double(lex.getNumber()) == 5.999);
	REQUIRE(lex.next() == ezcfg::Token::FLOAT);
	REQUIRE(double(lex.getNumber()) == 0xff.55ap-06);
	REQUIRE(lex.next() == ezcfg::Token::FLOAT);
	REQUIRE(double(lex.getNumber()) == .566e80);

	REQUIRE(lex.next() == ezcfg::Token::END);
}
