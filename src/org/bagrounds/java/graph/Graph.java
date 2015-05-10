package org.bagrounds.java.graph;

import com.sun.tools.javac.util.Pair;

import java.security.InvalidParameterException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Queue;


/**
 * Build a graph using generic, user-supplied objects as vertices and edges. Graph is capable of calculating unweighted
 * shortest paths between any two vertices that are members of the graph, assuming such a path exists.
 * <p>
 * Created by bryan on 4/29/15.
 */
public final class Graph < V, E > {

    private HashMap< V, Vertex< V > > vertices;
    private HashMap< Pair< V, V >, E > edges;
    private HashMap< E, Pair< V, V > > pairs;

    private Graph() {
        vertices = new HashMap<>();
        edges = new HashMap<>();
        pairs = new HashMap<>();
    }

    /**
     * Calls the private constructor and returns a new instance of Graph.
     *
     * @return a new instance of Graph
     */
    public static Graph newInstance() {
        return new Graph();
    }

    /**
     * Creates an edge going from the specified tail vertex to the specified head vertex. If the supplied vertices are
     * not already members of the graph, this method adds them first.
     *
     * @param tail the tail vertex of the provided edge
     * @param edge a new edge connecting the two provided vertices
     * @param head the head vertex of the provided edge
     *
     * @throws InvalidParameterException
     */
    public void connectVertices( V tail, E edge, V head ) throws InvalidParameterException {
        if ( tail == null || edge == null || head == null ) {
            throw new InvalidParameterException( "null vertices not allowed!" );
        }

        // add vertices to the graph
        addVertex( tail );
        addVertex( head );

        Vertex< V > t = vertices.get( tail );
        Vertex< V > h = vertices.get( head );
        t.addNeighbor( h );

        Pair< V, V > pair = hashKey( tail, head );
        edges.put( pair, edge );
        pairs.put( edge, pair );
    }

    /**
     * Adds the specified item to the graph as a vertex.
     *
     * @param item the object to be used as a vertex for this graph
     *
     * @throws java.security.InvalidParameterException if the item is null
     */
    public void addVertex( V item ) throws InvalidParameterException {
        if ( item == null ) {
            throw new InvalidParameterException( "null vertices not allowed!" );
        }

        if ( !vertices.containsKey( item ) ) {
            vertices.putIfAbsent( item, new Vertex<>( item ) );
        }
    }

    /**
     * Determines whether the specified vertex is a member of this graph instance or not.
     *
     * @param vertex the vertex whos membership is to be determined
     *
     * @return true if the vertex is a member of the graph, false otherwise.
     *
     * @throws java.security.InvalidParameterException if vertex is null
     */
    public boolean containsVertex( V vertex ) throws InvalidParameterException {
        if ( vertex == null ) {
            throw new InvalidParameterException( "null vertices not allowed!" );
        }

        return vertices.containsKey( vertex );
    }

    /**
     * Returns the vertex at which this edge points (the head).
     *
     * @param edge the edge whos head vertex is to be returned
     *
     * @return the vertex pointed to by this edge
     *
     * @throws InvalidParameterException
     */
    public V getHead( E edge ) throws InvalidParameterException {
        if ( edge == null ) {
            throw new InvalidParameterException( "null edges not allowed!" );
        }
        if ( !pairs.containsKey( edge ) ) {
            throw new InvalidParameterException( "edge must be a member of this graph!" );
        }

        return pairs.get( edge ).snd;
    }

    /**
     * Returns the vertex from which this edge points (the tail).
     *
     * @param edge the edge whos tail vertex is to be returned
     *
     * @return the vertex pointed to by this edge
     *
     * @throws InvalidParameterException
     */
    public V getTail( E edge ) throws InvalidParameterException {
        if ( edge == null ) {
            throw new InvalidParameterException( "null edges not allowed!" );
        }
        if ( !pairs.containsKey( edge ) ) {
            throw new InvalidParameterException( "edge must be a member of this graph!" );
        }

        return pairs.get( edge ).fst;
    }

    /**
     * Returns the edge connecting the specified vertices, assuming they are neighbors in this graph.
     *
     * @param v1 a vertex in this graph who has a neighbor v2
     * @param v2 a vertex in this graph who has a neighbor v1
     *
     * @return the edge connecting the specified vertices
     *
     * @throws InvalidParameterException if either specified vertex is null, or if they are not neighbors in this graph
     */
    public E getEdgeBetween( V v1, V v2 ) throws InvalidParameterException {
        if ( v1 == null || v2 == null ) {
            throw new InvalidParameterException( "null vertices not allowed!" );
        }
        if ( !edges.containsKey( hashKey( v1, v2 ) ) ) {
            throw new InvalidParameterException( "v1 and v2 vertices must be neighbors in this graph" );
        }

        return edges.get( hashKey( v1, v2 ) );
    }

    /**
     * Returns the edges comprising the unweighted shortest path between the specified source and sync vertices,
     * assuming they are connected prior to calling this method.
     *
     * @param source the vertex the computed path should start from
     * @param sync the vertex the computed path should end at
     *
     * @return an ArrayList containing the edges comprising the shortest path, in order, beginning at source
     *
     * @throws InvalidParameterException if either specified vertex is null or not present in the graph
     */
    public ArrayList< E > unweightedShortestEdgePath( V source, V sync ) throws InvalidParameterException {
        if ( source == null || sync == null ) {
            throw new InvalidParameterException( "null vertices not allowed!" );
        }
        if ( !vertices.containsKey( source ) || !vertices.containsKey( sync ) ) {
            throw new InvalidParameterException( "vertices must be a member of this graph!" );
        }

        ArrayList< E > edgePath = new ArrayList<>();
        ArrayList< V > vertexPath = unweightedShortestVertexPath( source, sync );

        for ( int i = 0; i < vertexPath.size() - 1; i++ ) {
            edgePath.add( getEdgeBetween( vertexPath.get( i ), vertexPath.get( i + 1 ) ) );
        }
        return edgePath;
    }

    /**
     * Returns the vertices comprising the unweighted shortest path between (inclusive) the specified source and sync
     * vertices, assuming they are connected prior to calling this method.
     *
     * @param source the vertex the computed path should start from
     * @param sync the vertex the computed path should end at
     *
     * @return an ArrayList containing the vertices comprising the shortest path, in order, beginning at source
     *
     * @throws InvalidParameterException if either specified vertex is null or not present in the graph
     */
    public ArrayList< V > unweightedShortestVertexPath( V source, V sync ) throws InvalidParameterException {
        if ( source == null || sync == null ) {
            throw new InvalidParameterException( "null vertices not allowed!" );
        }

        if ( !containsVertex( source ) || !containsVertex( sync ) ) {
            throw new InvalidParameterException( "vertex must be added to graph first!" );
        }


        Vertex< V > sourceVertex = vertices.get( source );
        Vertex< V > syncVertex = vertices.get( sync );

        resetSearchPaths();

        // set up breadth first search (BFS)

        boolean found = false;
        Vertex< V > cursor = sourceVertex;

        Queue< Vertex< V > > q = new ArrayDeque<>();
        q.add( cursor );

        // while the queue is not empty
        while ( !q.isEmpty() ) {
            // get vertex at front of queue and mark as visited
            cursor = q.poll();
            cursor.visited = true;

            // add all previously unvisited vertices to the queue
            for ( Vertex< V > v : cursor.neighbors ) {

                if ( !v.visited ) {
                    q.add( v );

                    // mark the current vertex (cursor) as the previous vertex in the path to this neighbor
                    v.previous = cursor;

                    // when sync vertex is found, we're done and can break early
                    if ( v.equals( sync ) ) {
                        found = true;
                        break;
                    } // end if
                } // end if
            } // end for

            if ( found ) {
                break;
            }
        }
        //if( ! found ) throw new InvalidParameterException("vertices: " + source + " and " + sync + " are not
        // connected!");

        // we've computed the shortest path, now we have to recover it by following previous references
        ArrayList< V > path = new ArrayList<>();
        cursor = syncVertex;

        while ( cursor != null ) {
            path.add( 0, cursor.item );
            cursor = cursor.previous;
        }

        return path;
    }

    private void resetSearchPaths() {
        vertices.values().forEach( Vertex::resetSearchData );
    }

    private Pair< V, V > hashKey( V v1, V v2 ) {
        return new Pair<>( v1, v2 );
    }

    private static class Vertex < A > {

        final A item;
        boolean visited; // used during search
        HashSet< Vertex< A > > neighbors;
        Vertex< A > previous; // previous vertex in a search path

        Vertex( A item ) {
            neighbors = new HashSet<>();
            this.item = item;
            this.visited = false;
            this.previous = null;
        }

        void addNeighbor( Vertex< A > neighbor ) {
            neighbors.add( neighbor );
        }

        void resetSearchData() {
            visited = false;
            previous = null;
        }

        @Override
        public String toString() {
            return "[" + item + "]";
        }
    }
}
