#ifndef FILE_OPERATION_HPP
#define FILE_OPERATION_HPP
#include <fstream>
#include <iostream>
#include <vector>
struct DataBuff
{
  char* data;
  long size;
};

// Common file related operations
class FileOperation
{
public:
  FileOperation();
  ~FileOperation();
  std::string setFileName(std::string file_type);
  struct DataBuff readFileAsDatabuffer(const std::string file_path);
  std::string readFileAsString(const std::string file_path);
  void getAllFilesName(std::string dir_path, std::vector<std::string>& files_name);

private:
  int test_file_id_ = 0;
};

// Read csv file data line by line
class CSVOperation
{
public:
  std::string const& operator[](std::size_t index) const
  {
    return m_data[index];
  }
  std::size_t size() const
  {
    return m_data.size();
  }
  void readNextRow(std::istream& str);

  std::vector<std::vector<std::string>> readAllCSV(std::istream& str);

  void writeRowData(std::string csv_file, std::vector<std::string> row_vector);

private:
  std::vector<std::string> m_data;
  std::vector<std::vector<std::string>> csv_table_;
};

#endif
