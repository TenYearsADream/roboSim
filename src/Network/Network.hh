#pragma once

#include <curl/curl.h>
#include "../../extern/json/json.hh"
#include "../Tests/Test.hh"

using json = nlohmann::json;

// #define URLPREFIX std::string("http://127.0.0.1:9999")
#define URLPREFIX std::string("http://s17839246.onlinehome-server.info:9999")

namespace Network {

char* exec(const char* command) {
  FILE* fp;
  char* line = NULL;
  // Following initialization is equivalent to char* result = ""; and just
  // initializes result to an empty string, only it works with
  // -Werror=write-strings and is so much less clear.
  char* result = (char*) calloc(1, 1);
  size_t len = 0;

  fflush(NULL);
  fp = popen(command, "r");
  if (fp == NULL) {
    printf("Cannot execute command:\n%s\n", command);
    return NULL;
  }

  while (getline(&line, &len, fp) != -1) {
    // +1 below to allow room for null terminator.
    result = (char*) realloc(result, strlen(result) + strlen(line) + 1);
    // +1 below so we copy the final null terminator.
    strncpy(result + strlen(result), line, strlen(line) + 1);
    free(line);
    line = NULL;
  }

  fflush(fp);
  if (pclose(fp) != 0) {
    perror("Cannot close stream.\n");
  }
  return result;
}

struct UploadParameter {
  std::string name, value;
  UploadParameter(std::string _name, std::string _value) : name(_name), value(_value) {}

  static UploadParameter fromJSON(std::string _name, json _value) {
    return UploadParameter(_name, _value.dump());
  }
};

static void upload(std::string _url, const std::vector<UploadParameter>& _parameters) {
  CURL* curl;
  CURLcode res;

  curl_global_init(CURL_GLOBAL_ALL);

  curl = curl_easy_init();
  if (curl) {
    curl_easy_setopt(curl, CURLOPT_URL, _url.c_str());

    std::string s = "";
    for (size_t i = 0; i < _parameters.size(); i++) {
      // static const char* postthis = "{\"aber=@\": \"hallo\", \"num\": 4}";
      char* escaped = curl_easy_escape(curl, _parameters[i].value.c_str(), 0);
      s += _parameters[i].name + "=" + std::string(escaped);
      if (i < _parameters.size() - 1)
        s += "&";
    }
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, s.c_str());

    res = curl_easy_perform(curl);
    if (res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

    curl_easy_cleanup(curl);
  }
  curl_global_cleanup();
}

static void upload(std::string _rev, std::string _branch, std::string _commitmessage, std::string _strategy, std::string _testname, std::vector<UploadParameter> _parameters) {
  _parameters.push_back(Network::UploadParameter("branch", _branch));
  _parameters.push_back(Network::UploadParameter("commitmessage", _commitmessage));
  _parameters.push_back(Network::UploadParameter("testName", _testname));
  upload(URLPREFIX + "/commit/" + _rev + "/strategy/" + _strategy + "/uploadData", _parameters);
}

static void upload(std::string _strategy, std::string _testname, const std::vector<UploadParameter>& _parameters) {
  std::string branch = exec("git rev-parse --abbrev-ref HEAD");
  std::string commitmessage = exec("git log -1 --format=%s -n 1");
  std::string revcommit = exec("git rev-parse --short HEAD");
  std::string revtime = exec("git show -s --format=%ct");

  branch.erase(std::remove(branch.begin(), branch.end(), '\n'), branch.end());
  commitmessage.erase(std::remove(commitmessage.begin(), commitmessage.end(), '\n'), commitmessage.end());
  revcommit.erase(std::remove(revcommit.begin(), revcommit.end(), '\n'), revcommit.end());
  revtime.erase(std::remove(revtime.begin(), revtime.end(), '\n'), revtime.end());

  std::string rev = revtime + "_" + revcommit;

  upload(rev, branch, commitmessage, _strategy, _testname, _parameters);
}

static void upload(Robot* _robot, const Test& _test) {
  Network::upload(_robot->getName(), _test.getName(), { Network::UploadParameter::fromJSON("data", _test.getData()) });
}

};


