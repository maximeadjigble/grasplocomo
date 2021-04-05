/**
 * @file
 * @brief Source file for dxFile
 * @author Maxime Adjigble
 * @author Naresh Marturi
 * @ref License
 */
 
/*
* BSD 3 - Clause License
*
* Copyright(c) 2021, Maxime Adjigble 
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met :
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#ifndef DX_FILE_INCLUDE
#define DX_FILE_INCLUDE
#include <iostream>
#include <vector>
#include <future>
#include <iterator>
#include <fstream>
#include <sstream>

template <typename T>
class dxFile
{
public:

	typedef std::vector<std::vector<T>> DataType;

	dxFile(std::string path, bool write = false)
	{
		setPath(path);
		this->isWriteMode = write;
	}

	virtual ~dxFile() {}

	void setPath(std::string path) {
		this->path = path;
	}

	void setLineParser(std::function<T(std::string)> parser) {
		this->parser = parser;
	}

	DataType getData() {
		return data;
	}

	std::vector<std::string> readLines() {
		std::vector<std::string> lines;
		std::ifstream f(path);

		if (!f.is_open()) {
			std::cout << "[ERROR] Opening the file: " << path << std::endl;
			return {};
		}

		std::string line;
		while (getline(f, line)) {
			lines.push_back(line);
		}
		f.close();

		return lines;
	}

	DataType readData() {
		if (parser == nullptr) {
			std::cout << "[Error] readData(): parser not set" << std::endl;
			return {};
		}

		data.clear();
		std::vector<std::string> lines = readLines();
		for (auto l : lines) {
			data.push_back(parseLine(l));
		}

		return data;
	}

	void printData() {
		for (auto line : data) {
			for (auto v : line) {
				std::cout << std::to_string(v) << " ";
			}
			std::cout << std::endl;
		}
	}

	std::vector<T> parseLine(std::string line) {
		std::istringstream iss(line);

		//Create a vector from the line
		std::vector<std::string> lineAsVec{ std::istream_iterator<std::string>{iss},
													  std::istream_iterator<std::string>{} };
		std::vector<T> lineData;
		if (parser == nullptr)
			return {};

		for (auto v : lineAsVec) {
			lineData.push_back(parser(v));
		}
		return lineData;
	}

	void write(std::vector<std::string> data, std::string header = "") {
		if (!isWriteMode) {
			std::cout << "[Error] writeData() file in read only mode" << std::endl;
		}

		std::ofstream f(path);
		if (header != "") {
			f << header << std::endl;
		}

		for (auto line : data) {
			f << line << std::endl;
		}
		f.close();
	}

protected:
	bool isWriteMode;
	std::string path;
	DataType data;
	std::function<T(std::string)> parser;
};

#endif