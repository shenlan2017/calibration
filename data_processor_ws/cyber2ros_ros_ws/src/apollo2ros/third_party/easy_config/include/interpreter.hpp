/********************************************************************\
|*  This file is part of easy_config.                               *|
|*                                                                  *|
|*  Copyright (c) 2021-2022 Incloon                                 *|
|*                                                                  *|
|* easy_config is free software : you can redistribute it and/or    *|
|* modify it under the terms of the GNU Lesser General Public       *|
|* License as published by the Free Software Foundation, either     *|
|* version 3 of the License, or (at your option) any later version. *|
|*                                                                  *|
|* easy_config is distributed in the hope that it will be useful,   *|
|* but WITHOUT ANY WARRANTY; without even the implied warranty of   *|
|* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the      *|
|* GNU Lesser General Public License for more details.              *|
|*                                                                  *|
|* You should have received a copy of the GNU Lesser General Public *|
|* License along with easy_config.                                  *|
|* If not, see < https://www.gnu.org/licenses/>.                    *|
\********************************************************************/
#pragma once

#include <type_traits>
#include <utility>
#include <lexer.hpp>

#include <string>
#include <array>
#include <vector>
#include <deque>
#include <list>
#include <forward_list>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>

namespace ezcfg
{
	class Interpreter
	{
	public:
		Interpreter() = default;

		Interpreter(const std::string& str, bool is_file = true)
			: lex{ str, is_file }
		{}

		bool loadFile(const std::string& file)
		{ return lex.loadFile(file); }

		bool loadSource(const std::string& source)
		{ return lex.loadSource(source); }

		template<typename T>
		typename std::enable_if<std::is_arithmetic<T>::value>::type parse(T& data)
		{
			if (lex.option(Token::L_BRACE))
			{
				if (lex.option(Token::R_BRACE))
					data = T{};
				else
				{
					data = exprExpr();
					lex.option(Token::COMMA);
					lex.match(Token::R_BRACE);
				}
			}
			else
				data = exprList();

			lex.option(Token::SEMICOLON);
		}

		template<class T>
		typename std::enable_if<!std::is_arithmetic<T>::value>::type parse(T& data)
		{
			parserDispatcher(data);
			lex.option(Token::SEMICOLON);
		}

		template<typename T, typename... TS>
		inline void parse(T& data, TS&... data_pkg)
		{
			parse(data);
			parse(data_pkg...);
		}

		inline ArithmeticT parseExpression()
		{
			auto temp = exprList();
			lex.option(Token::SEMICOLON);
			return temp;
		}

		explicit operator bool() const
		{ return static_cast<bool>(lex); }

	private:
		/*
		<list>    =   <expr> {"," <expr>}
		<expr>    =   <term> {("+" | "-") <term>}
		<term>    =   <factor> {("*" | "/" | "%") <factor>}
		<factor>  =   {("-" | "+")} <base>
		<base>    =   <constant> | "(" <list> ")"
		*/

		ArithmeticT exprList()
		{
			auto&& op = exprExpr();
			while (lex.option(Token::COMMA))
				op = exprExpr();
			return op;
		}

		ArithmeticT exprExpr()
		{
			auto&& l_op = exprTerm();
			while(true)
				switch (lex.getToken())
				{
				case Token::ADD:
					lex.next();
					l_op = l_op + exprTerm();
					break;
				case Token::SUB:
					lex.next();
					l_op = l_op - exprTerm();
					break;
				default:
					return l_op;
				}
		}

		ArithmeticT exprTerm()
		{
			auto&& l_op = exprFactor();
			while (true)
				switch (lex.getToken())
				{
				case Token::MUL:
					lex.next();
					l_op = l_op * exprFactor();
					break;
				case Token::DIV:
					lex.next();
					l_op = l_op / exprFactor();
					break;
				case Token::REM:
					lex.next();
					l_op = l_op % exprFactor();
					break;
				default:
					return l_op;
				}
		}

		ArithmeticT exprFactor()
		{
			size_t sub_symbol_count = 0;

			while (lex.getToken() == Token::ADD || lex.getToken() == Token::SUB)
			{
				if (lex.getToken() == Token::SUB)
					sub_symbol_count++;
				lex.next();
			}

			if(sub_symbol_count%2==1)
				return -exprBase();
			else
				return exprBase();
		}

		ArithmeticT exprBase()
		{
			if (lex.option(Token::L_PARENTHESIS))
			{
				auto&& temp = exprList();
				lex.match(Token::R_PARENTHESIS);
				return temp;
			}
			else
				return exprConst();
		}

		ArithmeticT exprConst()
		{
			if (lex.getToken() == Token::INT || lex.getToken() == Token::FLOAT)
			{
				auto&& temp = lex.getNumber();
				lex.next();
				return temp;
			}
			else
				lex.syntaxError("Expected number");
		}

		template<typename T>
		void parseArithmeticCell(T& num)
		{
			static_assert(std::is_arithmetic<T>::value, "Expected a arithmetic type");

			if (lex.option(Token::L_BRACE))
			{
				if (lex.option(Token::R_BRACE))
					num = T{};
				else
				{
					num = exprExpr();
					lex.option(Token::COMMA);
					lex.match(Token::R_BRACE);
				}
			}
			else
				num = exprExpr();
		}

		void stringToString(std::string& string)
		{
			string = lex.getTokenText();
			lex.match(Token::STR);
		}

		template<typename T, size_t n>
		void stringToString(T(&string)[n])
		{
			const std::string& raw = lex.getTokenText();

			if (raw.size() >= n)
				lex.syntaxError("initializer-string for current array is too long");

			for (size_t i = 0; i < raw.size(); ++i)
				string[i] = raw[i];
			string[raw.size()] = 0;

			lex.match(Token::STR);
		}

		void characterStreamToString(std::string& string)
		{
			char temp;
			parseArithmeticCell(temp);
			string.clear();
			string.push_back(temp);
			while (lex.getToken() == Token::COMMA)
			{
				if (lex.next() == Token::R_BRACE)
					return ;
				parseArithmeticCell(temp);
				string.push_back(temp);
			}
		}

		template<typename T, size_t n>
		void characterStreamToString(T(&string)[n])
		{
			parseArithmeticCell(string[0]);
			for (size_t i = 1; i < n; i++)
			{
				lex.match(Token::COMMA);
				parseArithmeticCell(string[i]);
			}
			lex.option(Token::COMMA);
		}

		template<typename T>
		void parseString(T& string)
		{
			if (lex.option(Token::L_BRACE))
			{
				if (lex.getToken() == Token::STR)
				{
					stringToString(string);
					lex.option(Token::COMMA);
				}
				else if (lex.getToken() != Token::R_BRACE)
				{
                    characterStreamToString(string);
				}
				lex.match(Token::R_BRACE);
			}
			else if (lex.getToken() == Token::STR)
				stringToString(string);
			else
				lex.syntaxError("Expected string");
		}

		template<size_t n, typename T>
		void parseArray(T& array)
		{
			static_assert(n, "Array size shouldn't be zero!");

			lex.match(Token::L_BRACE);
			parserDispatcher(array[0]);
			for (size_t i = 1; i < n; i++)
			{
				lex.match(Token::COMMA);
				parserDispatcher(array[i]);
			}
			lex.option(Token::COMMA);
			lex.match(Token::R_BRACE);
		}

		template<typename CellT, typename T>
		void parseDynamicArray(T& dynamic_array)
		{
			lex.match(Token::L_BRACE);
			typename std::decay<CellT>::type temp;
			dynamic_array.clear();
			if (lex.getToken() != Token::R_BRACE)
			{
				parserDispatcher(temp);
				dynamic_array.push_back(std::move(temp));
				while (lex.getToken() == Token::COMMA)
				{
					if (lex.next() == Token::R_BRACE)
						break;
					parserDispatcher(temp);
					dynamic_array.push_back(std::move(temp));
				}
			}
			lex.match(Token::R_BRACE);
		}

        template<typename CellT, typename T>
        void parseForwardList(T& forward_list)
        {
            lex.match(Token::L_BRACE);
            typename std::decay<CellT>::type temp;
            forward_list.clear();
            if (lex.getToken() != Token::R_BRACE)
            {
                parserDispatcher(temp);
                auto it = forward_list.insert_after(forward_list.before_begin(), std::move(temp));
                while (lex.getToken() == Token::COMMA)
                {
                    if (lex.next() == Token::R_BRACE)
                        break;
                    parserDispatcher(temp);
                    it = forward_list.insert_after(it, std::move(temp));
                }
            }
            lex.match(Token::R_BRACE);
        }

        template<typename CellT, typename T>
		void parseSet(T& set)
		{
			lex.match(Token::L_BRACE);
			typename std::decay<CellT>::type temp;
			set.clear();
			if (lex.getToken() != Token::R_BRACE)
			{
				parserDispatcher(temp);
				set.emplace(std::move(temp));
				while (lex.getToken() == Token::COMMA)
				{
					if (lex.next() == Token::R_BRACE)
						break;
					parserDispatcher(temp);
					set.emplace(std::move(temp));
				}
			}
			lex.match(Token::R_BRACE);
		}

		template<typename CellT1, typename CellT2, typename T>
		void parseMap(T& map)
		{
			lex.match(Token::L_BRACE);
			std::pair<CellT1, CellT2> pair;
			map.clear();
			if (lex.getToken() == Token::L_BRACE)
			{
				parserDispatcher(pair);
				map.emplace(std::move(pair));
				while (lex.getToken() == Token::COMMA)
				{
					if (lex.next() != Token::L_BRACE)
						break;
					parserDispatcher(pair);
					map.emplace(std::move(pair));
				}
			}
			lex.match(Token::R_BRACE);
		}



		template<class T>
		typename std::enable_if<!std::is_arithmetic<T>::value>::type parserDispatcher(T&);

		template<typename T>
		typename std::enable_if<std::is_arithmetic<T>::value>::type parserDispatcher(T& num)
		{ parseArithmeticCell(num); }



		template<typename T, size_t n>
		void parserDispatcher(T(&array)[n])
		{ parseArray<n>(array); }

		template<typename T1, size_t n>
		void parserDispatcher(std::array<T1, n>& array)
		{ parseArray<n>(array); }



		template<size_t n>
		void parserDispatcher(char(&string)[n])
		{ parseString(string); }

		template<size_t n>
		void parserDispatcher(signed char(&string)[n])
		{ parseString(string); }

		template<size_t n>
		void parserDispatcher(unsigned char(&string)[n])
		{ parseString(string); }

		void parserDispatcher(std::string& string)
		{ parseString(string); }



		template<typename T, typename A>
		void parserDispatcher(std::vector<T, A>& vector)
		{ parseDynamicArray<T>(vector); }

		template<typename T, typename A>
		void parserDispatcher(std::deque<T, A>& deque)
		{ parseDynamicArray<T>(deque); }

		template<typename T, typename A>
		void parserDispatcher(std::list<T, A>& list)
		{ parseDynamicArray<T>(list); }

		template<typename T, typename A>
		void parserDispatcher(std::forward_list<T, A>& forward_list)
		{ parseForwardList<T>(forward_list); }



		template<typename T, typename C, typename A>
		void parserDispatcher(std::set<T, C, A>& set)
		{ parseSet<T>(set); }

		template<typename T, typename C, typename A>
		void parserDispatcher(std::multiset<T, C, A>& set)
		{ parseSet<T>(set); }

		template<typename T, typename H, typename C, typename A>
		void parserDispatcher(std::unordered_set<T, H, C, A>& set)
		{ parseSet<T>(set); }

		template<typename T, typename H, typename C, typename A>
		void parserDispatcher(std::unordered_multiset<T, H, C, A>& set)
		{ parseSet<T>(set); }



		template<typename T1, typename T2>
		void parserDispatcher(std::pair<T1, T2>& pair)
		{
			lex.match(Token::L_BRACE);
			typename std::decay<T1>::type a;
			parserDispatcher(a);
			lex.match(Token::COMMA);
			typename std::decay<T2>::type b;
			parserDispatcher(b);
			lex.option(Token::COMMA);
			lex.match(Token::R_BRACE);
			pair = std::make_pair<T1, T2>(std::move(a), std::move(b));
		}

		template<typename T1, typename T2, typename C, typename A>
		void parserDispatcher(std::map<T1, T2, C, A>& map)
		{ parseMap<T1, T2>(map); }

		template<typename T1, typename T2, typename C, typename A>
		void parserDispatcher(std::multimap<T1, T2, C, A>& map)
		{ parseMap<T1, T2>(map); }

		template<typename T1, typename T2, typename H, typename C, typename A>
		void parserDispatcher(std::unordered_map<T1, T2, H, C, A>& map)
		{ parseMap<T1, T2>(map); }

		template<typename T1, typename T2, typename H, typename C, typename A>
		void parserDispatcher(std::unordered_multimap<T1, T2, H, C, A>& map)
		{ parseMap<T1, T2>(map); }



		Lexer lex;
	};
} /* namespace: ezcfg */
