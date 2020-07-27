#include "file_operation.h"
#include <dirent.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include "common_config.h"
#include <algorithm>
#include <cstring>
#include <sstream>
FileOperation::FileOperation(){};
FileOperation::~FileOperation(){};

// Name the file with test time,Time_t is accurate to the second.
std::string FileOperation::setFileName(std::string file_type)
{
  const time_t t = time(NULL);
  struct tm* systemtime = localtime(&t);
  std::string file_name = "/" + std::to_string(test_file_id_) + "-" + std::to_string(1900 + systemtime->tm_year) + "-" +
                          std::to_string(1 + systemtime->tm_mon) + "-" + std::to_string(systemtime->tm_mday) + "-" +
                          std::to_string(systemtime->tm_hour) + "-" + std::to_string(systemtime->tm_min) + "-" +
                          std::to_string(systemtime->tm_sec) + file_type;
  test_file_id_++;
  return file_name;
};

struct DataBuff FileOperation::readFileAsDatabuffer(const std::string file_path)
{
  struct DataBuff data_buff;
  std::ifstream file(file_path, std::ios::binary);

  if (file.is_open())
  {
    file.seekg(0, file.end);
    data_buff.size = file.tellg();

    file.seekg(0, file.beg);
    data_buff.data = new char[data_buff.size];
    file.read(data_buff.data, data_buff.size);
  }
  else
  {
    std::cout << "Fail to open expected audio file: " << file_path << std::endl;
    exit(ERROR_FILE_OPEN_FAIL);
  }
  file.close();
  return data_buff;
}

std::string FileOperation::readFileAsString(const std::string file_path)
{
  std::fstream file(file_path, std::ios::in | std::ios::binary);
  std::stringstream sstream;

  sstream << file.rdbuf();
  file.close();

  std::string str(sstream.str());
  sstream.clear();

  return str;
}

// 获取某目录下所有文件名
void FileOperation::getAllFilesName(std::string dir_path, std::vector<std::string>& files_name)
{
  DIR* dir;
  struct dirent* ptr;
  char base[1000];

  if ((dir = opendir(dir_path.c_str())) == NULL)
  {
    std::cout << ("Open dir error...");
    exit(ERROR_DIRECTORY_OPEN_FAIL);
  }

  while ((ptr = readdir(dir)) != NULL)
  {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
    {
      continue;  //跳过'.'和'..'两个目录
    }
    else if (ptr->d_type == 8)
    {
      files_name.push_back(ptr->d_name);
    }
    else if (ptr->d_type == 10)
    {
      continue;  // 跳过链接文件
    }
    else if (ptr->d_type == 4)
    {
      files_name.push_back(ptr->d_name);
    }
  }
  closedir(dir);
  //排序，按从小到大排序
  sort(files_name.begin(), files_name.end());
}

void CSVOperation::readNextRow(std::istream& str)
{
  std::string line;
  std::getline(str, line);
  std::stringstream lineStream(line);
  std::string cell;

  m_data.clear();
  while (std::getline(lineStream, cell, ','))
  {
    m_data.push_back(cell);
  }
  // This checks for a trailing comma with no data after it.
  if (!lineStream && cell.empty())
  {
    // If there was a trailing comma then add an empty element.
    m_data.push_back("");
  }
}

std::vector<std::vector<std::string>> CSVOperation::readAllCSV(std::istream& infile)
{
  std::string row;
  while (!infile.eof())
  {
    if (infile.bad() || infile.fail())
    {
      std::cout << "Fail to open the CSV file! " << std::endl;
      exit(ERROR_FILE_OPEN_FAIL);
    }
    readNextRow(infile);
    csv_table_.push_back(m_data);
  }
  return csv_table_;
}

void CSVOperation::writeRowData(std::string csv_file, std::vector<std::string> row_vector)
{
  int size = row_vector.size();
  std::ofstream outFile(csv_file, std::ios::app);
  if (outFile.is_open())
  {
    for (int i = 0; i < size; i++)
    {
      outFile << row_vector[i] << ',';
    }
    outFile << std::endl;
  }
  else
  {
    std::cout << "Fail to open the CSV file! " << std::endl;
    exit(ERROR_FILE_OPEN_FAIL);
  }
  outFile.close();
}
