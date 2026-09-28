from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import os
import nbformat
from nbclient import NotebookClient
from nbconvert import HTMLExporter


def execute(path):
    root=Path(path).parent.parent
    os.environ['JUPYTER_PATH']=str(root/'.venv/share/jupyter')
    notebook=nbformat.read(path,as_version=4)
    NotebookClient(notebook,timeout=900,kernel_name='dbf-stability',resources={'metadata':{'path':str(root)}}).execute()
    nbformat.write(notebook,path)
    html,_=HTMLExporter().from_notebook_node(notebook)
    out=root/'outputs/notebook_html';out.mkdir(parents=True,exist_ok=True)
    (out/(Path(path).stem+'.html')).write_text(html,encoding='utf8')
    print('Executed',Path(path).name,flush=True)
    return str(path)


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    with ProcessPoolExecutor(max_workers=2) as pool:
        for result in pool.map(execute,sorted((root/'notebooks').glob('*.ipynb'))):print(result,flush=True)
