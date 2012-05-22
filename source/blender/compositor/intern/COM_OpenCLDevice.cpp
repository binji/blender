/*
 * Copyright 2011, Blender Foundation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor: 
 *		Jeroen Bakker 
 *		Monique Dewanchand
 */

#include "COM_OpenCLDevice.h"
#include "COM_WorkScheduler.h"


OpenCLDevice::OpenCLDevice(cl_context context, cl_device_id device, cl_program program)
{
	this->device = device;
	this->context = context;
	this->program = program;
	this->queue = NULL;
}

bool OpenCLDevice::initialize()
{
	cl_int error;
	queue = clCreateCommandQueue(context, device, 0, &error);
	return false;
}

void OpenCLDevice::deinitialize()
{
	if (queue) {
		clReleaseCommandQueue(queue);
	}
}

void OpenCLDevice::execute(WorkPackage *work)
{
	const unsigned int chunkNumber = work->getChunkNumber();
	ExecutionGroup * executionGroup = work->getExecutionGroup();
	rcti rect;

	executionGroup->determineChunkRect(&rect, chunkNumber);
	MemoryBuffer ** inputBuffers = executionGroup->getInputBuffers(chunkNumber);
	MemoryBuffer * outputBuffer = executionGroup->allocateOutputBuffer(chunkNumber, &rect);

	executionGroup->getOutputNodeOperation()->executeOpenCLRegion(this->context, this->program, this->queue, &rect, chunkNumber, inputBuffers);
	
	executionGroup->finalizeChunkExecution(chunkNumber, inputBuffers);
	if (outputBuffer != NULL) {
		outputBuffer->setCreatedState();
	}
}