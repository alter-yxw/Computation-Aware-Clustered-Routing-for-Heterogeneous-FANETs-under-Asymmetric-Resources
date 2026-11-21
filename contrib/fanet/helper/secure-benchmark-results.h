/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef FANET_SECURE_BENCHMARK_RESULTS_H
#define FANET_SECURE_BENCHMARK_RESULTS_H

#include "ns3/abort.h"
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace ns3 { namespace secure_benchmark {
// SHA-256 is embedded to keep standalone runs and target delivery independent
// of Python, OpenSSL and an external freezing/manifest command.
class BenchmarkSha256
{
public:
  void Add(const void* data,size_t size)
  {
    const auto* p=static_cast<const uint8_t*>(data); m_length+=size;
    for(size_t i=0;i<size;++i) { m_buffer[m_used++]=p[i]; if(m_used==64){Block();m_used=0;} }
  }
  void Add(const std::string& text) { Add(text.data(),text.size()); }
  std::string Finish()
  {
    const uint64_t bits=m_length*8; const uint8_t one=0x80,zero=0;
    Add(&one,1); while(m_used!=56) Add(&zero,1);
    uint8_t end[8]; for(unsigned i=0;i<8;++i) end[7-i]=uint8_t(bits>>(8*i)); Add(end,8);
    std::ostringstream s; s<<std::hex<<std::setfill('0'); for(auto x:m_state) s<<std::setw(8)<<x; return s.str();
  }
  static std::string Digest(const std::string& text) { BenchmarkSha256 h; h.Add(text); return h.Finish(); }
  static std::string File(const std::string& path)
  {
    std::ifstream f(path,std::ios::binary); NS_ABORT_MSG_IF(!f,"Cannot hash build file "<<path);
    BenchmarkSha256 h; std::array<char,65536> b{};
    while(f){f.read(b.data(),b.size());h.Add(b.data(),static_cast<size_t>(f.gcount()));}
    NS_ABORT_MSG_IF(!f.eof(),"Build file read failed "<<path); return h.Finish();
  }
private:
  static uint32_t R(uint32_t x,unsigned n){return (x>>n)|(x<<(32-n));}
  void Block()
  {
    static constexpr uint32_t k[64]={
      0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
      0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
      0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
      0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
      0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
      0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
      0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
      0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    uint32_t w[64]{};
    for(unsigned i=0;i<16;++i) for(unsigned j=0;j<4;++j) w[i]=(w[i]<<8)|m_buffer[4*i+j];
    for(unsigned i=16;i<64;++i) w[i]=w[i-16]+(R(w[i-15],7)^R(w[i-15],18)^(w[i-15]>>3))+w[i-7]+(R(w[i-2],17)^R(w[i-2],19)^(w[i-2]>>10));
    auto v=m_state;
    for(unsigned i=0;i<64;++i){uint32_t t1=v[7]+(R(v[4],6)^R(v[4],11)^R(v[4],25))+((v[4]&v[5])^(~v[4]&v[6]))+k[i]+w[i];uint32_t t2=(R(v[0],2)^R(v[0],13)^R(v[0],22))+((v[0]&v[1])^(v[0]&v[2])^(v[1]&v[2]));for(unsigned j=7;j>0;--j)v[j]=v[j-1];v[4]+=t1;v[0]=t1+t2;}
    for(unsigned i=0;i<8;++i)m_state[i]+=v[i];
  }
  std::array<uint32_t,8> m_state{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
  std::array<uint8_t,64> m_buffer{};size_t m_used{0};uint64_t m_length{0};
};
inline std::string BenchmarkJson(const std::string& s)
{
  std::ostringstream out; out<<'"';
  for(unsigned char c:s){switch(c){case '"':out<<"\\\"";break;case '\\':out<<"\\\\";break;case '\n':out<<"\\n";break;case '\r':out<<"\\r";break;case '\t':out<<"\\t";break;default:if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;else out<<c;}}
  out<<'"';return out.str();
}
inline void BenchmarkAppend(const std::string& path,const std::string& line)
{
  int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_APPEND,0644);
  NS_ABORT_MSG_IF(fd<0,"Cannot append "<<path);
  NS_ABORT_MSG_IF(::flock(fd,LOCK_EX)!=0,"Cannot lock "<<path);
  size_t done=0; while(done<line.size()){ssize_t n=::write(fd,line.data()+done,line.size()-done);NS_ABORT_MSG_IF(n<=0,"Write failed "<<path);done+=n;}
  NS_ABORT_MSG_IF(::fsync(fd)!=0,"Flush failed "<<path);::close(fd);
}
inline std::string BenchmarkBuildId(std::string* manifest=nullptr)
{
  std::map<std::string,std::string> files;
  auto exe=std::filesystem::read_symlink("/proc/self/exe").string();
  files["entry-executable"]=BenchmarkSha256::File(exe);
  std::ifstream maps("/proc/self/maps"); NS_ABORT_MSG_IF(!maps,"Cannot read loaded build libraries");
  std::string line; std::set<std::string> paths;
  while(std::getline(maps,line)) { auto i=line.find('/'); if(i==std::string::npos) continue; std::string p=line.substr(i); if(p.find("libns3")!=std::string::npos) paths.insert(p); }
  for(const auto& p:paths) files[std::filesystem::path(p).filename().string()]=BenchmarkSha256::File(p);
  NS_ABORT_MSG_IF(paths.empty(),"No loaded ns-3 libraries in build binding");
  std::ostringstream text,json; json<<'{';bool first=true;
  for(const auto& [name,hash]:files){text<<name<<'='<<hash<<'\n';json<<(first?"":",")<<BenchmarkJson(name)<<':'<<BenchmarkJson(hash);first=false;}
  json<<'}'; if(manifest)*manifest=json.str();return BenchmarkSha256::Digest(text.str());
}

class SecureBenchmarkResults
{
public:
  using Row=std::vector<std::pair<std::string,std::string>>;
  std::string csv,journal,buildId,runnerId,pairKey,caseKey,config;
  std::vector<std::map<std::string,std::string>> saved;
  std::string partialTail;
  uint64_t completeBytes{0};
  static std::vector<std::string> Split(const std::string& text)
  {
    std::vector<std::string> fields; size_t start=0;
    for(size_t i=0;i<=text.size();++i)
      if(i==text.size()||text[i]==','){fields.push_back(text.substr(start,i-start));start=i+1;}
    return fields;
  }
  static std::string Csv(const Row& values,bool header)
  {
    std::ostringstream s;bool first=true;for(auto& p:values){const auto& v=header?p.first:p.second;NS_ABORT_MSG_IF(v.find_first_of(",\r\n\"")!=std::string::npos,"Unsafe CSV field");s<<(first?"":",")<<v;first=false;}return s.str();
  }
  static std::string JsonObject(const Row& values)
  { std::ostringstream s;s<<'{';bool first=true;for(const auto& [k,v]:values){s<<(first?"":",")<<BenchmarkJson(k)<<':'<<(v=="N/A"?"null":BenchmarkJson(v));first=false;}s<<'}';return s.str(); }
  void Read(const Row& prototype)
  {
    if(csv.empty())return;
    std::ifstream f(csv); if(!f)return;
    std::string header; if(!std::getline(f,header))return;
    NS_ABORT_MSG_IF(header!=Csv(prototype,true),"CSV schema mismatch; choose a NEW CSV, historical evidence is preserved: "<<csv);
    auto columns=Split(header);std::string line;
    completeBytes=header.size()+1;
    while(std::getline(f,line))
    {
      // checkOnly is read-only. A later normal invocation preserves this tail
      // as evidence and repairs only this writer's already verified schema.
      if(f.eof()){partialTail=line;break;}
      auto fields=Split(line);NS_ABORT_MSG_IF(fields.size()!=columns.size(),"CSV column count mismatch "<<csv);
      std::map<std::string,std::string> row;for(size_t i=0;i<columns.size();++i)row[columns[i]]=fields[i];saved.push_back(std::move(row));
      completeBytes+=line.size()+1;
    }
  }
  bool Completed() const
  {for(const auto& r:saved)if(r.at("CaseKey")==caseKey && r.at("Accepted")=="1")return true;return false;}
  std::map<std::string,std::string> Baseline() const
  {
    for(auto it=saved.rbegin();it!=saved.rend();++it)if(it->at("PairKey")==pairKey && it->at("Attack")=="none" && it->at("Accepted")=="1")return *it;
    return {};
  }
  void Start(const std::string& manifest,const std::string& command)
  {
    if(journal.empty())return;
    BenchmarkAppend(journal,"{\"event\":\"start\",\"case_key\":"+BenchmarkJson(caseKey)+",\"pair_key\":"+BenchmarkJson(pairKey)+",\"build_id\":"+BenchmarkJson(buildId)+",\"runner_id\":"+BenchmarkJson(runnerId)+",\"config\":"+BenchmarkJson(config)+",\"command\":"+BenchmarkJson(command)+",\"build_manifest\":"+manifest+"}\n");
  }
  void Commit(const Row& row,const std::string& raw)
  {
    std::string header=Csv(row,true),line=Csv(row,false);
    // Raw evidence, including the exact CSV row, is durable before the CSV
    // commit. An interrupted append is recoverable from this journal in C++.
    if(!journal.empty())BenchmarkAppend(journal,"{\"event\":\"run\",\"case_key\":"+BenchmarkJson(caseKey)+",\"row_sha256\":"+BenchmarkJson(BenchmarkSha256::Digest(line))+",\"csv_row\":"+BenchmarkJson(line)+",\"results\":"+JsonObject(row)+",\"raw\":"+raw+"}\n");
    if(!csv.empty())
    {
      if(!std::filesystem::exists(csv)||std::filesystem::file_size(csv)==0)BenchmarkAppend(csv,header+"\n");
      BenchmarkAppend(csv,line+"\n");
    }
  }
  // Replay only an intact C++ run record whose CSV commit is absent. Strings
  // below are generated by this writer; no general JSON/Python parser is used.
  bool Recover(const Row& prototype)
  {
    if(journal.empty())return false;
    if(!partialTail.empty())
    {
      BenchmarkAppend(journal,"{\"event\":\"interrupted_csv_tail\",\"offset\":"+
          std::to_string(completeBytes)+",\"preserved_text\":"+BenchmarkJson(partialTail)+"}\n");
      int fd=::open(csv.c_str(),O_WRONLY);
      NS_ABORT_MSG_IF(fd<0 || ::flock(fd,LOCK_EX)!=0,"Cannot repair interrupted CSV");
      NS_ABORT_MSG_IF(::ftruncate(fd,completeBytes)!=0 || ::fsync(fd)!=0,"Cannot flush interrupted CSV repair");
      ::close(fd);partialTail.clear();
    }
    if(Completed())return false;
    std::ifstream in(journal);std::string line,found;
    const std::string key="\"case_key\":"+BenchmarkJson(caseKey);
    while(std::getline(in,line))
    {
      if(in.eof()||line.find("\"event\":\"run\"")==std::string::npos||line.find(key)==std::string::npos)continue;
      auto p=line.find("\"csv_row\":\"");if(p==std::string::npos)continue;p+=11;
      std::string row;bool escaped=false;
      for(;p<line.size();++p){char c=line[p];if(escaped){row+=c;escaped=false;}else if(c=='\\')escaped=true;else if(c=='"')break;else row+=c;}
      const std::string digest="\"row_sha256\":"+BenchmarkJson(BenchmarkSha256::Digest(row));
      auto fields=Split(row);auto columns=Split(Csv(prototype,true));if(fields.size()!=columns.size()||line.find(digest)==std::string::npos)continue;
      auto accepted=std::find(columns.begin(),columns.end(),"Accepted")-columns.begin();
      if(fields.at(accepted)=="1")found=row;
    }
    if(found.empty())return false;
    if(!std::filesystem::exists(csv)||std::filesystem::file_size(csv)==0)BenchmarkAppend(csv,Csv(prototype,true)+"\n");
    BenchmarkAppend(csv,found+"\n");saved.clear();Read(prototype);return true;
  }
};
}} // ns3::secure_benchmark
#endif
