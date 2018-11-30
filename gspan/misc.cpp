/*
 *  misc.cpp
 *  GSPAN
 *
 *  Created by Jinseung KIM on 09. 07. 19.
 *  Copyright 2009 KyungHee. All rights reserved.
 *
 */

#include "gspan.h"

using namespace std;
const RMPath& DFSCode::buildRMPath() //buildRMPath là một phương thức của DFSCODE
{
	rmpath.clear();
	int old_from = -1;
	for(int i = size() -1;i>=0;--i) //Duyệt qua tất cả các cạnh của DFS_CODE theo thứ tự từ cạnh cuối đến cạnh đầu (Duyệt ngược)
	{
		if ((*this)[i].from < (*this)[i].to && (rmpath.empty() || old_from == (*this)[i].to))
		{
			rmpath.push_back(i);
			old_from = (*this)[i].from;
		}
	}
	
	return rmpath;
}

void History::build(Graph& g,PDFS* e) //đưa vào id của đồ thị và một cạnh
{
	clear(); //cái này là đối tượng history kế thừa từ vector<Edge*>, nên lệnh clear này là dọn dẹp vector<Edge*> của history.
	edge.clear(); //vector<int> edge
	edge.resize(g.edge_size());// thay đổi kích thước của vector<int> edge bằng với số lượng cạnh trong đồ thị đang xét
	vertex.clear(); //xoá tất cả các đỉnh trong vector<int> vertex
	vertex.resize(g.size()); //thay đổi kích thước của vector<int> vertex bẳng số lượng đỉnh trong đồ thị đang xét
	
	if(e){ //e ở đây chính là tham số thứ 2 A_1, e là một con trỏ lưu trử địa chỉ của A_1
		push_back(e->edge);		//Đưa embedding đang xét vào vector<Edge*> vào history
		edge[e->edge->id] = vertex[e->edge->from] = vertex[e->edge->to] = 1; //đồng thời đánh dấu các cạnh và đỉnh của g liên quan đến Embedding đó hay nói cách khác là bậc 1 cho đỉnh cạnh đã duyệt Embedding
		
		for(PDFS* p = e->prev; p; p=p->prev){
			push_back(p->edge);
			edge[p->edge->id] = vertex[p->edge->from] = vertex[p->edge->to] = 1; //đánh dấu các cạnh trước đó đã kết nối với embedding đang xét.
		}
		
		std::reverse(begin(),end());
	}
}

//Cạnh mở rộng bắt buộc phải lớn hơn tất cả các cạnh kề với đỉnh mở rộng (chỉ cần xét lớn hơn hoặc bằng với cạnh trên right most path phía sau đỉnh mở rộng là đủ)
bool get_forward_rmpath (Graph &graph, Edge *e, int minlabel, History& history, EdgeList &result) //Tìm các mở rộng cho các đỉnh còn lại trên Right Most Path, ngoại trừ đỉnh cuối cùng của DFS_CODE
{
	result.clear ();


	
	int tolabel = graph[e->to].label; //nhãn đỉnh to của cạnh right most path đang xét
	
	for (Vertex::edge_iterator it = graph[e->from].edge.begin() ; //Duyệt qua các cạnh mở rộng với đỉnh From của right most path
		 it != graph[e->from].edge.end() ; ++it)
	{
		int tolabel2 = graph[it->to].label; //Lấy nhãn đỉnh to của cạnh mở rộng
		if (e->to == it->to || minlabel > tolabel2 || history.hasVertex (it->to))//Loại bỏ mở rộng khi: id đỉnh to của mở rộng bằng với id đỉnh to của cạnh right most path, vì lúc này cạnh mở rộng trùng với right most path
			continue; // hoặc nhãn đỉnh to của mở rộng nhỏ hơn nhãn đỉnh minlabel hoặc id của đỉnh to của mở rộng đã thuộc DFS_CODE rồi.
		
		if (e->elabel < it->elabel || (e->elabel == it->elabel && tolabel <= tolabel2)) //nhãn cạnh của right most path nhỏ hơn nhãn cạnh mở rộng, hoặc nhãn cạnh của right most path bằng với nhãn cạnh mở rộng và nhãn đỉnh to của righ most path nhỏ hơn hoặc bằng nhãn đỉnh to của mở rộng.
			result.push_back (&(*it));
	}
	
	return (! result.empty());
}


//Tìm các mở rộng forward từ đỉnh phải cùng của right most path
//Tại sao nó không so sánh nhãn cạnh của mở rộng với nhãn cạnh kết nối với đỉnh from của mở rộng?
bool get_forward_pure (Graph &graph, Edge *e, int minlabel, History& history, EdgeList &result)
{
	result.clear ();
	for (Vertex::edge_iterator it = graph[e->to].edge.begin() ; //bắt đầu mở rộng cạnh từ đỉnh to, lưu ý đỉnh này phải thuộc right most path
		 it != graph[e->to].edge.end() ; ++it) //ở đây là duyệt qua tất cả các cạnh kề với đỉnh to trên right most path
	{
		if (minlabel > graph[it->to].label || history.hasVertex (it->to)) //Nếu đỉnh của cạnh muốn thêm đã thuộc DFS_CODE hoặc nhãn đỉnh mới < minlabel của DFS_CODE thì bỏ qua cạnh đó
			continue; //bỏ qua cạnh đó.
		
		result.push_back (&(*it));
	}
	
	return (! result.empty());
}

bool get_forward_root (Graph &g, Vertex &v, EdgeList &result) //Xây dựng các forward ban đầu
{
	result.clear (); //xóa tất cả các phần tử trong result, tức là làm rỗng result.
	for (Vertex::edge_iterator it = v.edge.begin(); it != v.edge.end(); ++it) { //duyệt qua tất cả các cạnh của đỉnh v đang xét
		if (v.label <= g[it->to].label) //nếu như nhãn của đỉnh v nhỏ hơn nhãn đỉnh 'to' kề với nó thì lưu cạnh đó vào EdgeList &result.
			result.push_back (&(*it)); //it là một edge_iterator, về mặt kỹ thuật nó là một con trỏ, nó trỏ đến tất cả các cạnh kết nối với đỉnh v.
	}
	
	return (! result.empty()); //nếu result không rỗng tức là tồn tại forward_edge
}

//The như giải pháp ở mục 5.1 của bài báo gspan thì một mở rộng backward được gọi là hợp lệ nếu như cạnh mở rộng lớn hơn tất cả các cạnh nối với đỉnh "to"
//của cạnh mở rộng. Ở đây e1 thuộc right most path, nên e1 là cạnh lớn nhất trong tất cả các cạnh kề với đỉnh "to" của backward extension, nên chúng ta chỉ cần so 
//sánh backward extension với e1 là đủ.
Edge *get_backward (Graph &graph, Edge* e1, Edge* e2, History& history)//ts1: đồ thị, ts2: e1 là một cạnh thuộc RMPath (bắt đầu duyệt từ cạnh đầu tiên đến cạnh gần cuối) của DFS_CODE, ts3: e2 là cạnh cuối của DFS_CODE, ts4: history cho biết những cạnh nào của đồ thị đã thuộc DFS_CODE để khi mở rộng chúng ta bỏ qua những cạnh đó.
{
	if (e1 == e2)	//Nếu DFS_CODE chỉ có 1 cạnh có nghĩa là cạnh đầu bằng cạnh cuối (e1=e2) thì chúng ta return 0, tức là không có backward edge nào
		return 0;
	
	for (Vertex::edge_iterator it = graph[e2->to].edge.begin() ; //Duyệt qua tất cả các cạnh kề với đỉnh phải nhất của embeddings của DFS_CODE
		 it != graph[e2->to].edge.end() ; ++it)
	{
		if (history.hasEdge (it->id)) //Nếu cạnh đó thuộc embedding rồi thì bỏ qua cạnh đó và xét cạnh khác
			continue;
		
		if ( (it->to == e1->from) &&			//Nếu đỉnh to của cạnh mở rộng bằng với đỉnh from của e1 và
			( (e1->elabel < it->elabel) ||		//nhãn cạnh của mở rộng lớn hơn nhãn cạnh của e1 hoặc
			 (e1->elabel == it->elabel) &&		// nếu nhãn cạnh của mở rộng bằng với nhãn cạnh của e1 thì 
			 (graph[e1->to].label <= graph[e2->to].label) //nhãn đỉnh to của e1 nhỏ hơn bằng nhãn đỉnh to của e2
			 ) )
		{
			return &(*it);
		}
	}
	
	return 0;
}
