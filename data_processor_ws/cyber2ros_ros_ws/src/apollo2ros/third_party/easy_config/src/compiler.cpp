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

#define COMPILER
#include <lexer.hpp>
#include <vector>

namespace ezcfg
{
	class Compiler
	{
		struct StructMemberInfo
		{
			std::string identify_name;
			std::string identify_type;
			bool have_default_value = false;
		};

	public:
		Compiler(const std::string& file)
		{
			auto ofs_ptr = std::unique_ptr<std::ofstream>{ new std::ofstream(file) };
			if (!ofs_ptr->is_open())
				exit(-1);
			stream = std::move(ofs_ptr);
		}

		Compiler(int argc, char* argv[])
			: Compiler(argv[argc - 1])
		{
			loadFile(argc - 2, argv + 1);
		}

		void loadFile(int argc, char** argv)
		{
			for (size_t i = 1; i < argc - 1; i++)
				file_list.emplace_back(argv[i]);
			loadFile(argv[argc - 1]);
		}

		void loadFile(const std::string& file)
		{
			if (!lex.loadFile(file))
				exit(-1);
		}

		void loadSource(const std::string& source)
		{
			if (!lex.loadSource(source))
				exit(-1);
		}

		void templateArgumentsRecognize()
		{
			lex.match(Token::L_ANGLE_BRACKET);

			std::string& type_name = struct_info.back().identify_type;
			type_name.push_back('<');
			while(true)
				switch (lex.getToken())
				{
				case Token::L_ANGLE_BRACKET:
					templateArgumentsRecognize();
					break;
				case Token::L_BRACKET:
					while (lex.getToken() == Token::L_BRACKET)
					{
						lex.match(Token::L_BRACKET);
						type_name.push_back('[');
						type_name += std::to_string(size_t(lex.getNumber()));
						lex.match(Token::INT);
						lex.match(Token::R_BRACKET);
						type_name.push_back(']');
					}
					break;
				case Token::ID:
					type_name += lex.getTokenText();
					lex.next();
					break;
				case Token::SCOPE:
					lex.next();
					type_name += "::";
					break;
				case Token::COMMA:
					lex.next();
					type_name += ", ";
					break;
				case Token::R_ANGLE_BRACKET:
					lex.next();
					type_name.push_back('>');
					return;
				default:
					lex.syntaxError("declare type error");
					break;
				}
		}

		bool variableDeclaration()
		{
			struct_info.emplace_back();
			std::string& identify = struct_info.back().identify_name;
			std::string& type_name = struct_info.back().identify_type;

			while(true)
				switch (lex.getToken())
				{
				case Token::SEMICOLON:
					lex.next();
					break;
				default:
					struct_info.pop_back();
					return false;
					break;
				case Token::ID:
					type_name = lex.getTokenText();
					if (lex.next() == Token::ID)
					{
						identify = lex.getTokenText();
						while (lex.next() == Token::ID)
						{
							type_name.push_back(' ');
							type_name += identify;
							identify = lex.getTokenText();
						}
					}
					if(lex.getToken() == Token::SCOPE || lex.getToken() == Token::L_ANGLE_BRACKET)
						type_name += identify;

					while (lex.getToken() == Token::SCOPE)
					{
				case Token::SCOPE:
						lex.next();
						type_name += "::";
						type_name += lex.getTokenText();
						lex.match(Token::ID);
					}
					if (lex.getToken() == Token::L_ANGLE_BRACKET)
						templateArgumentsRecognize();

					if (lex.getToken() == Token::ID)
					{
						identify = lex.getTokenText();
						lex.next();
					}
					while (lex.getToken() == Token::L_BRACKET)
					{
						lex.match(Token::L_BRACKET);
						type_name.push_back('[');
						type_name += std::to_string(size_t(lex.getNumber()));
						lex.match(Token::INT);
						lex.match(Token::R_BRACKET);
						type_name.push_back(']');
					}
					if (lex.option(Token::EQU) || lex.getToken() == Token::L_BRACE)
					{
						struct_info.back().have_default_value = true;
						while (lex.next() != Token::SEMICOLON);
					}
					lex.match(Token::SEMICOLON);
					return true;
				}
		}

		void structDeclaration()
		{
			lex.match(Token::STRUCT);
			current_scope_name.push_back(lex.getTokenText());
			lex.match(Token::ID);
			lex.match(Token::L_BRACE);
			while (variableDeclaration());
			lex.match(Token::R_BRACE);
			lex.match(Token::SEMICOLON);
			genSructParseCode();
			current_scope_name.pop_back();
		}

		void namespaceDeclaration()
		{
			lex.match(Token::NAMESPACE);
			current_scope_name.push_back(lex.getTokenText());
			lex.match(Token::ID);
			lex.match(Token::L_BRACE);
			while(true)
				switch (lex.getToken())
				{
				case Token::NAMESPACE:
					namespaceDeclaration();
					break;
				case Token::STRUCT:
					structDeclaration();
					break;
                case Token::R_BRACE:
                    lex.match(Token::R_BRACE);
                    current_scope_name.pop_back();
                    return;
				default:
					lex.syntaxError("Unexpected token");
					break;
				}
		}

		void genSructParseCode()
		{
			if (struct_info.empty())
				lex.syntaxError("Empty struct is not support!");

			*stream << "template<> void Interpreter::parserDispatcher<";
			std::stringstream scope_name;
			for (auto& smi : current_scope_name)
				scope_name << "::" << smi;
			*stream << scope_name.str() << ">(" << scope_name.str() << "& data)\n";

			*stream << "{\n\tlex.match(Token::L_BRACE);\n";
			bool is_in_if = false;

			auto rit = struct_info.rbegin();
			for (; rit != struct_info.rend(); ++rit)
				if (!rit->have_default_value)
					break;

			auto it = struct_info.begin();
			if (it != rit.base())
			{
				for (; it < rit.base() - 1; ++it)
				{
					if (is_in_if) *stream << "\t";
					*stream << "\tlex.match(Token::DOT);\n";
					if (is_in_if) *stream << "\t}\n";
					is_in_if = it->have_default_value;
					if (is_in_if)
					{
						*stream << "\tif(lex.getTokenText() == \"" << it->identify_name << "\")\n\t{\n";
						if (is_in_if) *stream << "\t";
						*stream << "\tlex.match(Token::ID);\n";
					}
					else
						*stream << "\tlex.matchID(\"" << it->identify_name << "\");\n";
					if (is_in_if) *stream << "\t";
					*stream << "\tif (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.match(Token::EQU);\n";
					if (is_in_if) *stream << "\t";
					*stream << "\tparserDispatcher(data." << it->identify_name << ");\t//" << it->identify_type << "\n";
					if (is_in_if) *stream << "\t";
					*stream << "\tlex.match(Token::COMMA);\n";
				}
				if (is_in_if) *stream << "\t";
				*stream << "\tlex.match(Token::DOT);\n";
				if (is_in_if) *stream << "\t}";
				is_in_if = false;
				*stream << "\tlex.matchID(\"" << it->identify_name << "\");\n";
				*stream << "\tif (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.match(Token::EQU);\n";
				*stream << "\tparserDispatcher(data." << it->identify_name << ");\t//" << it->identify_type << "\n";
				if (rit == struct_info.rbegin())
					*stream << "\tlex.option(Token::COMMA);\n";
				++it;
			}

			if (rit != struct_info.rbegin())
			{
				*stream << "\tbool matched = true;\n";
				*stream << "\tif (!(lex.option(Token::COMMA) && lex.option(Token::DOT))) goto end_parse;\n";
				*stream << "\telse matched = false;\n";
				for (; it < struct_info.end(); ++it)
				{
					*stream << "\tif(lex.getTokenText() == \"" << it->identify_name << "\")\n\t{\n";
					*stream << "\t\tlex.match(Token::ID);\n";
					*stream << "\t\tif (!lex.option(Token::EQU) && lex.getToken() != Token::L_BRACE) lex.match(Token::EQU);\n";
					*stream << "\t\tparserDispatcher(data." << it->identify_name << ");\t//" << it->identify_type << "\n";
					*stream << "\t\tmatched = true;\n";
					if (it == struct_info.end() - 1)
						*stream << "\t\tlex.option(Token::COMMA);\n\t}\n";
					else
					{
						*stream << "\t\tif (!(lex.option(Token::COMMA) && lex.option(Token::DOT))) goto end_parse;\n";
						*stream << "\t\telse matched = false;\n\t}\n";
					}
				}
				*stream << "\tend_parse: if(!matched) lex.match(Token::ID);\n";
			}

			*stream << "\tlex.match(Token::R_BRACE);\n}" << std::endl;
			struct_info.clear();
		}

		void compile()
		{
			*stream << "#include <interpreter.hpp>\n";
			if (lex.currentFilename() != "string")
			{
				*stream << "#include <" << lex.currentFilename() << ">\n";
				if (!file_list.empty())
					for (auto& file_name : file_list)
						*stream << "#include <" << file_name << ">\n";
			}

            *stream << "\nnamespace ezcfg\n{\n";

            while (true)
				switch (lex.getToken())
				{
				case Token::NAMESPACE:
					namespaceDeclaration();
					break;
				case Token::STRUCT:
					structDeclaration();
					break;
				case Token::END:
					if (file_list.empty())
                    {
                        *stream << "} /* namespace: ezcfg */\n";
                        return;
                    }
					else
					{
						loadFile(file_list.back());
						file_list.pop_back();
					}
				default:
					lex.syntaxError("Unexpected token");
					break;
				}
        }

	private:
		Lexer lex;
		std::vector<std::string> file_list;
		std::vector<StructMemberInfo> struct_info;
		std::vector<std::string> current_scope_name;
		std::unique_ptr<std::ostream> stream;
	};
}

int main(int argc, char** argv)
{
	ezcfg::Compiler cp(argc, argv);
	cp.compile();
	return 0;
}
