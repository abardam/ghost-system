#include <Windows.h>
#include <string>

#include "tinyxml.h"
#include "definitions.h"


void SaveStatusRecord(std::string path, std::vector<std::vector<char>>& goodRecord){
	std::string filepath = path + "/skelestatus.xml";
	CreateDirectoryA(path.c_str(), NULL);

	TiXmlDocument xmlDoc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	xmlDoc.LinkEndChild(decl);

	TiXmlElement * root = new TiXmlElement("SkeletonStatus");
	xmlDoc.LinkEndChild(root);
	root->SetDoubleAttribute("version", 0.1);
	root->SetAttribute("numjoints", NUMJOINTS);

	TiXmlElement * framesElement = new TiXmlElement("Frames");
	root->LinkEndChild(framesElement);
	framesElement->SetAttribute("numframes", goodRecord.size());

	for(int i=0;i<goodRecord.size();++i){
		TiXmlElement * frame = new TiXmlElement("Frame");
		framesElement->LinkEndChild(frame);
		frame->SetAttribute("frameid", i);

		for(int j=0;j<NUMJOINTS;++j){
			TiXmlElement * joint = new TiXmlElement("Joint");
			frame->LinkEndChild(joint);
			joint->SetAttribute("jointid", j);
			joint->SetAttribute("status", goodRecord[i][j]);
		}
	}

	xmlDoc.SaveFile(filepath);
}

void LoadStatusRecord(std::string path, std::vector<std::vector<char>>& goodRecord){
	std::string filepath = path + "/skelestatus.xml";
	TiXmlDocument xmlDoc;
	
	if(!xmlDoc.LoadFile(filepath)){
		std::cerr << "Could not read " << filepath << "; load failed\n";
		return;
	}

	TiXmlHandle doc(&xmlDoc);
	TiXmlElement * root = doc.FirstChildElement().ToElement();

	if(!root){
		std::cerr << "No root handle; load failed\n";
		return;
	}

	double version;
	int numframes;
	int numjoints;

	root->QueryDoubleAttribute("version", &version);
	root->QueryIntAttribute("numjoints", &numjoints);

	TiXmlElement * framesElement = root->FirstChild("Frames")->ToElement();
	TiXmlHandle framesHandle(framesElement);

	framesElement->QueryIntAttribute("numframes", &numframes);

	goodRecord.clear();
	goodRecord.resize(numframes);
	
	for(int i=0;i<numframes;++i){
		goodRecord[i].resize(numjoints);
	}

	for(TiXmlElement * frame = framesHandle.FirstChild().ToElement();
		frame != NULL;
		frame = frame->NextSiblingElement()){
			int frameid;
			frame->QueryIntAttribute("frameid", &frameid);

			TiXmlHandle frameHandle(frame);

			int jid=0;

			for(TiXmlElement * joint = frameHandle.FirstChild().ToElement();
				joint != NULL;
				joint = joint->NextSiblingElement()){
					int jointid;
					int status;
					joint->QueryIntAttribute("status", &status);

					if(joint->QueryIntAttribute("jointid", &jointid) == TIXML_NO_ATTRIBUTE){
						jointid = jid;
					}

					goodRecord[frameid][jointid] = status;

					++jid;
			}
	}
}