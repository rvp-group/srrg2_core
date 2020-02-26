#include <memory>

#include "blob.h"
#include "object_data.h"
#include "serialization_context.h"
#include "id_context.h"

using namespace srrg2_core;
using namespace std;

static string DEFAULT_EXTENSION("dat");

const string& BLOB::extension() {
  return DEFAULT_EXTENSION;
}

const string& BaseBLOBReference::extension() {
  if (_instance) {
    return _instance->extension();
  }
  return DEFAULT_EXTENSION;
}

void BaseBLOBReference::serialize(ObjectData& data, IdContext& context) {
  Identifiable::serialize(data, context);
  if (_instance) {
    //Check if binary file serialization is supported

    SerializationContext* fileContext = context.serializationContext();
    if (fileContext) {
      _fileName = fileContext->createBinaryFilePath(*this);
      std::ostream* os = fileContext->getBinaryOutputStream(_fileName);
      if (os) {
        _instance->write(*os);
        std::ofstream* os_fstream = dynamic_cast<std::ofstream*>(os);
        if (os_fstream)
          os_fstream->close();
        delete os;
      }
    }
  }
  data << field("path_name", _fileName);
}

void BaseBLOBReference::deserialize(ObjectData& data, IdContext& context) {
  Identifiable::deserialize(data, context);
  data >> field("path_name", _fileName);
  //cerr << "Path name: " << _fileName << endl;
  _instance = 0;
}

bool BaseBLOBReference::load(BLOB& instance) const {
  //Check if binary file serialization is supported
//  cerr << "Load: Path name: " << _fileName << endl;
//  cerr << "instance: " << &instance << endl;

  const SerializationContext* fileContext = getContext()->serializationContext();
  bool result = false;
  if (fileContext) {
    std::istream* is = fileContext->getBinaryInputStream(_fileName);
//    cerr << "istream: " << is << endl;
    if (is) {
      result = instance.read(*is);
      delete is;
    }
  }
  return result;
}

void BaseBLOBReference::set(BLOB* instance) {
  _instance.reset(instance);
}
