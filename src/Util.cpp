/*********************************************************************************
*  Copyright (c) 2011-2012, Paul Tschirhart
*                             Peter Enns
*                             Jim Stevens
*                             Ishwar Bhati
*                             Mu-Tien Chang
*                             Bruce Jacob
*                             University of Maryland 
*                             pkt3c [at] umd [dot] edu
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/

#include "Util.h"

uint64_t convert_uint64_t(string value)
{
	// Check that each character in value is a digit.
	for(size_t i = 0; i < value.size(); i++)
	{
		if(!isdigit(value[i]))
		{
			cout << "ERROR: Non-digit character found: " << value << "\n";
			abort();
		}
	}

	// Convert it
	stringstream ss;
	uint64_t var;
	ss << value;
	ss >> var;

	return var;
}

//mostly shamelessly stolen from HybridSim
void convert_uint64_t(uint64_t &var, string value, string infostring)
{
	// Check that each character in value is a digit.
	for(size_t i = 0; i < value.size(); i++)
	{
		if(!isdigit(value[i]))
		{
			cout << "ERROR: Non-digit character found: " << infostring << " : " << value << "\n";
			abort();
		}
	}

	// Convert it
	stringstream ss;
	ss << value;
	ss >> var;
}

void convert_bool(bool &var, string value, string infostring)
{
	// Check that each character in value is a digit.
	for(size_t i = 0; i < value.size(); i++)
	{
		if(!isdigit(value[i]))
		{
			cout << "ERROR: Non-digit character found: " << infostring << " : " << value << "\n";
			abort();
		}
	}

	// Convert it
	stringstream ss;
	ss << value;
	ss >> var;
}

void convert_float(float &var, string value, string infostring)
{
	// Convert it
	stringstream ss;
	ss << value;
	ss >> var;
}

//used to divide the ini parameters so that they don't result in a zero
//old version from when all ini params were unit not uint64_t
//preserved in case something else needs it
uint divide_params(uint num, uint denom)
{
    uint temp = (uint)(((float)num / (float)denom) + 0.99f);
    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

//used to divide the ini parameters so that they don't result in a zero

uint divide_params(uint64_t num, uint denom)
{
    uint temp = (uint)(((float)num / (float)denom) + 0.99f);
    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint divide_params(uint num, uint64_t denom)
{
    uint temp = (uint)(((float)num / (float)denom) + 0.99f);
    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint divide_params(uint64_t num, uint64_t denom)
{
    uint temp = (uint)(((float)num / (float)denom) + 0.99f);
    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint divide_params(float num, float denom)
{
    uint temp = (uint)((num / denom) + 0.99f);
    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint64_t divide_params_64b(uint64_t num, uint denom)
{
    stringstream ss;
    uint64_t temp;
    ss << (((float)num / (float)denom) + 0.99f);
    ss >> temp;

    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }

    return temp;
}

uint64_t divide_params_64b(uint num, uint64_t denom)
{
    stringstream ss;
    uint64_t temp;
    ss << (((float)num / (float)denom) + 0.99f);
    ss >> temp;

    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint64_t divide_params_64b(uint64_t num, uint64_t denom)
{
    stringstream ss;
    uint64_t temp;
    ss << (((float)num / (float)denom) + 0.99f);
    ss >> temp;

    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint64_t divide_params_64b(uint64_t num, float denom)
{
    stringstream ss;
    uint64_t temp;
    ss << (((float)num / denom) + 0.99f);
    ss >> temp;

    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint64_t divide_params_64b(float num, float denom)
{
    stringstream ss;
    uint64_t temp;
    ss << (uint)((num / denom) + 0.99f);
    ss >> temp;

    if(num <= 0)
    {
	    return 0;
    }
    else if(temp <= 0)
    {
	return 1;
    }
    
    return temp;
}

uint64_t subtract_params(uint64_t a, uint64_t b)
{
    if(a < b)
    {
	return 0;
    }
    else
    {
	return (a - b);
    }
}


// parsing helper functions stolen shamelessly from Hybridsim
string strip(string input, string chars)
{
	size_t pos;

	// Strip front.
	pos = input.find_first_not_of(chars);
	input.erase(0, pos);

	// Strip back.
	pos = input.find_last_not_of(chars);
	input.erase(pos+1);

	return input;
}


list<string> split(string input, string chars, size_t maxsplit)
{
	list<string> ret;

	string cur = input;

	size_t pos = 0;

	if (maxsplit == 0)
		maxsplit = 1;

	while (!cur.empty())
	{
		if (ret.size() == (maxsplit-1))
		{
			ret.push_back(cur);
			return ret;
		}

		pos = cur.find_first_of(chars);
		string tmp = cur.substr(0, pos);
		ret.push_back(tmp);


		if (pos == string::npos)
			cur.erase();
		else
		{
			// Skip ahead to the next non-split char
			size_t new_pos = cur.find_first_not_of(chars, pos);
			//cur.erase(0, pos+1);
			cur.erase(0, new_pos);
		}
	}

	// If not at npos, then there is still an extra empty string at the end.
	if (pos != string::npos)
	{
		ret.push_back("");
	}

	return ret;
}
