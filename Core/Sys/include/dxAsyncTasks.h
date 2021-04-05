/**
 * @file
 * @brief Source file for dxAsyncTasks
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
#ifndef DX_ASYNC_TASK_INCLUDE
#define DX_ASYNC_TASK_INCLUDE

#include <future>
#include <vector>

template <typename Params, typename Result>
class dxAsyncTasks {
public:
	typedef std::future<Result> Task;
	typedef std::vector<Task> TaskPool;
	typedef std::vector<Result> ResultVec;
	typedef std::function<Result(Params)> AsyncFunc;

	dxAsyncTasks() {}

	void addTask(Params params) {
		tasks.push_back(std::async(std::launch::async, asynFunction, params));
	}

	int getNum() {
		return tasks.size();
	}

	void setAsyncFunction(AsyncFunc f) {
		asynFunction = f;
	}

	void wait() {
		for (auto &task : tasks)
			task.wait();
	}

	ResultVec get() {
		std::vector<Result> results;
		for (auto &task : tasks)
			results.emplace_back(task.get());
		return results;
	}

	ResultVec getResults() {
		wait();
		return get();
	}

protected:
	TaskPool tasks;
	AsyncFunc asynFunction;
};

#endif