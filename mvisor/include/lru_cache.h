/*
 * Reference: https://blog.csdn.net/gaixm/article/details/104662991/
 *
 */

#ifndef _MVISOR_LRU_CACHE_H
#define _MVISOR_LRU_CACHE_H

#include <map>
#include <list>
#include <functional>
#include <memory>

template<typename TKey, typename TValue>
class SimpleLRUCache
{
private:
	size_t m_iMaxSize;
	std::list<TKey> m_listLru;
	
	typedef std::pair<typename std::list<TKey>::iterator, TValue> MPair;
	typedef std::shared_ptr<MPair> PairPtr;
	std::map<TKey, PairPtr> m_mapPair;
	std::function<void (TKey&, TValue&)> m_removeCallback;
 
public:
	const std::list<TKey>& list() const { return m_listLru; }
	const std::map<TKey, PairPtr>& map() const { return m_mapPair; }

	SimpleLRUCache(size_t iMaxSize = 128)
	{
		m_iMaxSize = iMaxSize;
	}

	void Initialize(size_t iMaxSize, std::function<void (TKey&, TValue&)> cb) {
		m_iMaxSize = iMaxSize;
		m_removeCallback = cb;
	}

	void Clear() {
		while (m_listLru.begin() != m_listLru.end()) {
			auto key = *m_listLru.begin();
			Remove(key);
		}
	}
 
	bool Contains(TKey& szKey)
	{
		auto iterFind = m_mapPair.find(szKey);
		if (iterFind == m_mapPair.end())
			return false;
		return true;
	}
 
	bool Get(TKey& szKey, TValue &rValue)
	{
		auto iterFind = m_mapPair.find(szKey);
		if (iterFind == m_mapPair.end())
			return false;
 
		rValue = iterFind->second->second;
 
		auto iterList = iterFind->second->first;
		m_listLru.erase(iterList);
		m_listLru.push_front(iterFind->first);
		iterFind->second->first = m_listLru.begin();
 
		return true;
	}
 
	bool Put(TKey& szKey, TValue& szValue)
	{
		if (Contains(szKey))
			return false;
 
		m_listLru.push_front(szKey);
		auto iterFront = m_listLru.begin();
		PairPtr pairPtr = std::make_shared<MPair>(iterFront, szValue);
		m_mapPair.insert(std::make_pair(szKey, pairPtr));
 
		if (m_listLru.size() > m_iMaxSize)
		{
			auto myKey = m_listLru.back();
			Remove(myKey);
		}
 
		return true;
	}
 
	bool Remove(TKey &szKey)
	{
		auto iterFind = m_mapPair.find(szKey);
		if (iterFind == m_mapPair.end())
			return false;
		
		if (m_removeCallback) {
			m_removeCallback(szKey, iterFind->second->second);
		}
		auto iterList = iterFind->second->first;
		m_listLru.erase(iterList);
		m_mapPair.erase(iterFind);
 
		return true;
	}
};


#endif // _MVISOR_LRU_CACHE_H
