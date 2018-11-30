/*
 *  ismin.cpp
 *  GSPAN
 *
 *  Created by Jinseung KIM on 09. 07. 19.
 *  Copyright 2009 KyungHee. All rights reserved.
 *
 */


#include "gspan.h"
using namespace std;	
bool gSpan::is_min () //Hàm này kiểm tra xem DFS_code đó có phải là min hay không
{
	if (DFS_CODE.size() == 1) //nếu như trong vector<DFS> chỉ có duy nhất 1 DFS thì nó là nhỏ nhất.
		return (true);
	//Nếu trong DFS_Code có nhiều cạnh thì làm các bước sau: Chuyển DFS_code đó sang đồ thị.
	DFS_CODE.toGraph (GRAPH_IS_MIN);  //xây dựng đồ thị cho DFS_CODE và gán đồ thị đó cho GRAPH_IS_MIN
	DFS_CODE_IS_MIN.clear ();
	
	Projected_map3 root;
	EdgeList           edges;
	
	for (unsigned int from = 0; from < GRAPH_IS_MIN.size() ; ++from) //Duyệt qua các đỉnh của đồ thị vừa mới xây dựng
		if (get_forward_root (GRAPH_IS_MIN, GRAPH_IS_MIN[from], edges)) //Tại mỗi đỉnh của GRAPH_IS_MIN tìm các forward edge hay nói cách khác là 
			for (EdgeList::iterator it = edges.begin(); it != edges.end();  ++it) //tìm forwardEdge tại mỗi đỉnh của DFS_CODE, kết quả lưu vào EdgeList edges
				root[GRAPH_IS_MIN[from].label][(*it)->elabel][GRAPH_IS_MIN[(*it)->to].label].push (0, *it, 0); //đưa tất cả các forward edge vào ánh xạ map3 gọi là root
	
	Projected_iterator3 fromlabel = root.begin();
	Projected_iterator2 elabel    = fromlabel->second.begin();
	Projected_iterator1 tolabel   = elabel->second.begin();
	
	DFS_CODE_IS_MIN.push (0, 1, fromlabel->first, elabel->first, tolabel->first); //Cạnh nhỏ nhất luôn là cạnh đầu tiên trong ánh xạ map3. Đưa cạnh đó vào DFS_CODE_MIN
	
	return (project_is_min (tolabel->second)); //từ tolabel->second tức các mở rộng tính từ đỉnh to của cạnh nhỏ nhất, ta có thể tiếp tục kiểm tra xem toàn bộ DFS_CODE có phải là nhỏ nhất hay không.
}

bool gSpan::project_is_min (Projected &projected) //với tham số là các mở rộng của tolabel.second()
{
	const RMPath& rmpath = DFS_CODE_IS_MIN.buildRMPath (); //xây dựng right most path cho DFS_CODE_IS_MIN
	int minlabel         = DFS_CODE_IS_MIN[0].fromlabel;
	int maxtoc           = DFS_CODE_IS_MIN[rmpath[0]].to; //là id của đỉnh cuối cùng trên rmpath của DFS_CODE_MIN
	//1.Phát sinh DFS_CODE_IS_MIN từ GRAPH_IS_MIN và so sánh nó với DFS_CODE từ các mở rộng backward của GRAPH_IS_MIN
	{
		Projected_map1 root; //<int,projected>
		bool flg = false; //nếu tìm được mở rộng backward hoặc forward, thì bậc flag=true để xây dựng DFS_CODE_IS_MIN từ cạnh mở rộng nhỏ nhất
		int newto = 0;
		//Nếu DFS_Code_Min có nhiều hơn 1 cạnh và flg=true thì mới tìm các backward
		for (int i = rmpath.size()-1; ! flg  && i >= 1; --i) { //duyệt qua các cạnh của rmpath, bắt đầu từ cạnh cuối cùng cho đến cạnh kề với cạnh cuối cùng của DFS_code
			for (unsigned int n = 0; n < projected.size(); ++n) { //duyệt qua các embeddings
				PDFS *cur = &projected[n];
				History history (GRAPH_IS_MIN, cur);
				Edge *e = get_backward (GRAPH_IS_MIN, history[rmpath[i]], history[rmpath[0]], history); //tìm tất cả các backward của DFS_Code
				if (e) {
					root[e->elabel].push (0, e, cur); //xây dựng ánh xạ root cho các backward vừa tìm được
					newto = DFS_CODE_IS_MIN[rmpath[i]].from;
					flg = true;
				}
			}
		}
		
		if (flg) {
			Projected_iterator1 elabel = root.begin(); //lấy backward nhỏ nhất.
			DFS_CODE_IS_MIN.push (maxtoc, newto, -1, elabel->first, -1); //đưa cạnh backwardEdge đó vào DFS_CODE_IS_MIN
			if (DFS_CODE[DFS_CODE_IS_MIN.size()-1] != DFS_CODE_IS_MIN [DFS_CODE_IS_MIN.size()-1]) return false;
			return project_is_min (elabel->second);
		}
	}
	//2.Tìm DFS_CODE_IS_MIN từ GRAPH_IS_MIN và so sanh với DFS_CODE từ các mở rộng forwardEdge của GRAPH_IS_MIN
	{
		bool flg = false;
		int newfrom = 0;
		Projected_map2 root; //<int,<int,<projected>>>
		EdgeList edges;
		
		for (unsigned int n = 0; n < projected.size(); ++n) {
			PDFS *cur = &projected[n];
			History history (GRAPH_IS_MIN, cur);
			if (get_forward_pure (GRAPH_IS_MIN, history[rmpath[0]], minlabel, history, edges)) { //Tìm forwardEdge. Nếu tìm được thì gán flag = true. Nếu không tìm được forwardEdge nào từ đỉnh cuối trên RMP thì tìm trên các đỉnh còn lại trên RMP.
				flg = true;
				newfrom = maxtoc; //cập nhật lại local id của newfrom
				for (EdgeList::iterator it = edges.begin(); it != edges.end();  ++it)
					root[(*it)->elabel][GRAPH_IS_MIN[(*it)->to].label].push (0, *it, cur); //Với mỗi forwardEdge tìm được ta đi xây dựng ánh xạ root.
			}
		}
		
		for (int i = 0; ! flg && i < (int)rmpath.size(); ++i) {
			for (unsigned int n = 0; n < projected.size(); ++n) {
				PDFS *cur = &projected[n];
				History history (GRAPH_IS_MIN, cur);
				if (get_forward_rmpath (GRAPH_IS_MIN, history[rmpath[i]], minlabel, history, edges)) {//Tìm các mở rộng từ right most path của DFS_Is_Min
					flg = true;
					newfrom = DFS_CODE_IS_MIN[rmpath[i]].from;
					for (EdgeList::iterator it = edges.begin(); it != edges.end();  ++it)
						root[(*it)->elabel][GRAPH_IS_MIN[(*it)->to].label].push (0, *it, cur);
				}
			}
		}
		
		if (flg) {
			Projected_iterator2 elabel  = root.begin(); //lấy nhãn cạnh nhỏ nhất trong các mở rộng forward vừa tìm được ở trên
			Projected_iterator1 tolabel = elabel->second.begin();
			DFS_CODE_IS_MIN.push (newfrom, maxtoc + 1, -1, elabel->first, tolabel->first); //lấy forwardEdge nhỏ nhất vừa tìm được để đưa nó vào DFS_CODE_IS_MIN 
			if (DFS_CODE[DFS_CODE_IS_MIN.size()-1] != DFS_CODE_IS_MIN [DFS_CODE_IS_MIN.size()-1]) 
				return false; //so sánh DFS_CODE và DFS_CODE_IS_MIN nếu chúng bằng nhau thì DFS_CODE là nhỏ nhất và tiếp tục xây dựng DFS_CODE_MIN để so sánh tiếp, ngược lại thì trả về false tức DFS_CODE không phải là nhỏ nhất.
			return project_is_min (tolabel->second); //tiếp tục xây dựng DFS_CODE_IS_MIN và so sánh với DFS_CODE.
		}
	}
	
	return true;
}

