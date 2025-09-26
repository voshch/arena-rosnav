from tqdm import tqdm
import time
from google import genai
import chromadb
from chromadb.api.types import EmbeddingFunction, Documents, Embeddings
from typing import Dict, List
import json


EMBEDDING_MODEL = "gemini-embedding-001"
EMBEDDING_CFG = genai.types.EmbedContentConfig(
    task_type="retrieval_document", 
    output_dimensionality=768
)
COLLECTION_NAME = "bt-nodes-reference"

class GeminiEmbeddingFunction(EmbeddingFunction):
    def __init__(self, client: genai.Client, model, config: genai.types.EmbedContentConfig, *args, **kwargs):
        self.client = client
        self.model = model
        self.config = config
              
    def __call__(self, input: Documents) -> Embeddings:
        response = self.client.models.embed_content(
            model=self.model,
            contents=input,
            config=self.config
		)
        return [item.values for item in response.embeddings]
    

def process_json_doc(doc_path:str) -> List[str]:
    with open(doc_path, 'rt') as f:
        data = json.load(f)

    documents = []

    for category, items in data["Node categories"].items():
        for node in items:
            doc = ""
            doc = f"\nNode name: {node['name']}\nPurpose: {node['purpose']}"
            doc += "\nInputs:"
            for input in node["inputs"]:
                input_name = input["name"]
                input_type = input["type"]
                input_description = input["description"]
                doc += f"\n\t- {input_name} ({input_type}): {input_description}"
            doc += "\nOutputs:"
            for output in node["outputs"]:
                output_name = output["name"]
                output_type = output["type"]
                output_description = output["description"]
                doc += f"\n\t- {output_name} ({output_type}): {output_description}"
            doc += f"\nMetadata:\n\t-Category: {category}"
            documents.append(doc)

    return documents
    

def create_chroma_db(documents: List[str], db_path: str, client: genai.Client, collection_name: str=COLLECTION_NAME, embedding_model: str=EMBEDDING_MODEL, embedding_config: genai.types.EmbedContentConfig=EMBEDDING_CFG) -> chromadb.Collection:
    chroma_client = chromadb.PersistentClient(db_path)
    
    collection = chroma_client.create_collection(
        name=collection_name, 
        embedding_function=GeminiEmbeddingFunction(
            client=client, 
            model=embedding_model, 
            config=embedding_config
        )
    )

    initiali_size = collection.count()
    for i, d in tqdm(enumerate(documents), total=len(documents), desc=f"Creating Chroma Collection at {db_path}"):
        collection.add(
            documents=d,
            ids=str(i + initiali_size)
        )
        time.sleep(0.5)

    return collection


def get_chroma_collection(db_path: str, client: genai.Client, collection_name: str=COLLECTION_NAME, embedding_model: str=EMBEDDING_MODEL, embedding_config: genai.types.EmbedContentConfig=EMBEDDING_CFG) -> chromadb.Collection:
    chroma_client = chromadb.PersistentClient(db_path)

    return chroma_client.get_collection(
        name=collection_name, 
        embedding_function=GeminiEmbeddingFunction(
            client=client, 
            model=embedding_model, 
            config=embedding_config
        )
    )


def get_relevant_bt_nodes(query:str, collection:chromadb.Collection, n_results=10) -> str:
    passages = collection.query(query_texts=[query], n_results=n_results)[
        'documents'][0]
    
    nodes_descriptions = ""
    
    for p in passages:
        nodes_descriptions += f'\n{p}'
    
    return nodes_descriptions.encode("utf-8").decode("unicode_escape").strip()