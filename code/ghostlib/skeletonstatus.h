#pragma once
#include <string>
#include <vector>

#define JT_GOOD 0
#define JT_BAD 1
#define JT_GOOD_to_BAD 2
#define JT_BAD_to_GOOD 3
#define JT_N 4

void SaveStatusRecord(std::string path, std::vector<std::vector<char>>& goodRecord);
void LoadStatusRecord(std::string path, std::vector<std::vector<char>>& goodRecord);