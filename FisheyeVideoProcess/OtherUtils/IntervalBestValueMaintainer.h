#include <list>
#include <map>
template <typename ItemType,typename ValueType>
class IntervalBestValueMaintainer {
private:
	std::pair<int,int> range;
	std::list<std::pair<std::pair<ItemType, ValueType>,int>> storage;
	std::map<int, typename std::list<std::pair<std::pair<ItemType, ValueType>,int>>::iterator > idxMap;
	
	int bestNum;
	int intervalLen;
	ValueType (*getValue)(const ItemType &) ;

public:
	IntervalBestValueMaintainer(int _bnum=1,int intvlLen=1, ValueType (*gv)(const ItemType &)=NULL)
		:bestNum(_bnum),intervalLen(intvlLen) {
		storage.clear();
		idxMap.clear();
		getValue = gv;
	}
	int getCandNum() const {return range.second - range.first;}
	std::pair<int,int> getRange() const {return range;}
	bool isCandCovered(int l, int r) const {return range.first <= l && range.second >= r;}
	friend std::ostream& operator <<(std::ostream& out, const IntervalBestValueMaintainer& obj) {
		out << "<IntervalBestValueMaintainer> {";
		for (auto id:idxMap) {
			out << id.first << ",";
		}
		out << "} <\\IntervalBestValueMaintainer>";
	}

	bool addCandidate(int fidx, ItemType& g) {
		if (fidx != range.second) {
			LOG_ERR("IntervalBestValueMaintainer: Inconsistency occurs when adding candidates.");
			LOG_ERR(this);
			return false;
		}
		range.second++;

		// first evict item outside range
		if (range.second - range.first > intervalLen) range.first++;
		int preSize = idxMap.size();
		while (!idxMap.empty() && (*idxMap.begin()).first < range.first) {
			auto it = (*idxMap.begin()).second;
			storage.erase(it);
			idxMap.erase(idxMap.begin());
		}
		if (preSize - idxMap.size() > 1) {
			LOG_ERR(
				"IntervalBestValueMaintainer: Inconsistency occurs when adding candidates (evicting the old).");
			LOG_ERR(this);
			return false;
		}
		if (idxMap.size() < bestNum) {
			// Add new candidates
			storage.push_back(std::make_pair(std::make_pair(g, getValue(g)),fidx));
			idxMap[fidx] = --(storage.end());
		} else {
			// 1) Add new candidates
			ValueType val = getValue(g);
			for (auto it = storage.begin(); it != storage.end(); ++it) {
				if ((*it).first.second < val) {
					idxMap[fidx] = storage.insert(it, std::make_pair(std::make_pair(g, val), fidx));
					break;
				}
			}
			// 2) evict least one
			if (idxMap.size() > bestNum) {
				idxMap.erase((*(--(storage.end()))).second);
				storage.pop_back();
			}
		}
		return true;
	}

	ItemType & operator[](int fidx) {
		if (idxMap.find(fidx) != idxMap.end()) {
			return (*(idxMap[fidx])).first.first;
		} else {
			return ItemType();
		}
	}

	std::vector<std::pair<int, ValueType>> getBestIdx(int head, int tail) {
		std::vector<std::pair<int, ValueType>> ret;
		if (head == range.first && tail == range.second) {
			for (auto item:storage) {
				ret.push_back(std::make_pair(item.second, item.first.second));
			}

		} else {
			LOG_ERR("IntervalBestValueMaintainer: Inconsistency when getBestidx()");
			LOG_ERR("Required:[" << head<<","<<tail<<"); Actual:[" << range.first << "," << range.second<<")");
		}
		return ret;
	}
};